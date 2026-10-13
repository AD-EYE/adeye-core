#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

class SteeringFaultProxy
{
public:
    SteeringFaultProxy() : nh_(), private_nh_("~")
    {
        private_nh_.param("input_topic", input_topic_,
                          std::string("/nominal/steering_requested_phy"));
        private_nh_.param("output_topic", output_topic_,
                          std::string("/steering_requested_phy"));
        private_nh_.param("publish_rate_hz", publish_rate_hz_, 20.0);
        private_nh_.param("fault_timeout_s", fault_timeout_s_, 5.0);
        private_nh_.param("max_fault_timeout_s", max_fault_timeout_s_, 300.0);
        private_nh_.param("max_input_age_s", max_input_age_s_, 0.3);
        private_nh_.param("max_steering_angle_rad", max_angle_rad_, 0.85);
        private_nh_.param("max_steering_offset_rad", max_offset_rad_,
                          0.017453292519943295);
        private_nh_.param("max_steering_saturation_rad", max_saturation_rad_,
                          0.17453292519943295);
        private_nh_.param("max_steering_oscillation_rad", max_oscillation_rad_,
                          0.008726646259971648);
        private_nh_.param("oscillation_frequency_hz", oscillation_frequency_hz_,
                          1.0);
        private_nh_.param("max_recovery_rate_rad_s", max_recovery_rate_rad_s_,
                          0.2);
        private_nh_.param("enable_fault_injection", enable_fault_injection_, true);
        private_nh_.param("enable_fault_timeout", enable_fault_timeout_, false);
        private_nh_.param("zero_on_source_loss", zero_on_source_loss_, false);
        private_nh_.param("source_loss_timeout_s", source_loss_timeout_s_, 2.0);

        if (!parametersValid()) {
            ROS_FATAL("Steering proxy settings are not valid");
            valid_configuration_ = false;
            return;
        }

        output_pub_ = nh_.advertise<std_msgs::Float64>(output_topic_, 1);
        status_pub_ = nh_.advertise<std_msgs::String>(
            "/adeye/physical_fault_status", 10
        );
        inhibit_pub_ = nh_.advertise<std_msgs::Bool>(
            "/adeye/steering_output_inhibit", 1, true
        );
        input_sub_ = nh_.subscribe<std_msgs::Float64>(
            input_topic_, 1, &SteeringFaultProxy::inputCallback, this
        );
        if (enable_fault_injection_) {
            command_sub_ = nh_.subscribe<std_msgs::String>(
                "/vehicle_commands", 10, &SteeringFaultProxy::commandCallback, this
            );
            config_sub_ = nh_.subscribe<std_msgs::String>(
                "/adeye/physical_fault_config", 10,
                &SteeringFaultProxy::configCallback, this
            );
        }
        timer_ = nh_.createWallTimer(
            ros::WallDuration(1.0 / publish_rate_hz_),
            &SteeringFaultProxy::timerCallback, this
        );
        last_timer_time_ = ros::WallTime::now();
        publishInhibit(false);
        ROS_INFO("Steering fault proxy: %s -> %s at %.3f Hz",
                 input_topic_.c_str(), output_topic_.c_str(), publish_rate_hz_);
        ROS_INFO("Steering fault injection: %s",
                 enable_fault_injection_ ? "enabled" : "disabled (normal commands only)");
    }

    bool validConfiguration() const { return valid_configuration_; }

private:
    enum class State { PASS_THROUGH, FAULT_ACTIVE, RECOVER, INHIBITED };
    enum class Fault { NONE, OFFSET, FREEZE, SATURATION, OSCILLATION };

    bool parametersValid() const
    {
        return std::isfinite(publish_rate_hz_) && publish_rate_hz_ > 0.0 &&
            std::isfinite(fault_timeout_s_) && fault_timeout_s_ > 0.0 &&
            std::isfinite(max_fault_timeout_s_) && max_fault_timeout_s_ > 0.0 &&
            fault_timeout_s_ <= max_fault_timeout_s_ &&
            std::isfinite(max_input_age_s_) && max_input_age_s_ > 0.0 &&
            std::isfinite(max_angle_rad_) && max_angle_rad_ > 0.0 &&
            std::isfinite(max_offset_rad_) && max_offset_rad_ > 0.0 &&
            std::isfinite(max_saturation_rad_) && max_saturation_rad_ > 0.0 &&
            max_saturation_rad_ <= max_angle_rad_ &&
            std::isfinite(max_oscillation_rad_) && max_oscillation_rad_ > 0.0 &&
            std::isfinite(oscillation_frequency_hz_) &&
            oscillation_frequency_hz_ > 0.0 &&
            std::isfinite(max_recovery_rate_rad_s_) &&
            max_recovery_rate_rad_s_ > 0.0 &&
            std::isfinite(source_loss_timeout_s_) &&
            source_loss_timeout_s_ > 0.0;
    }

    bool parseValue(const std::string& text, const std::string& prefix,
                    double* value) const
    {
        if (text.compare(0, prefix.size(), prefix) != 0) {
            return false;
        }
        try {
            const std::string value_text = text.substr(prefix.size());
            std::size_t consumed = 0;
            const double parsed = std::stod(value_text, &consumed);
            if (consumed != value_text.size() || !std::isfinite(parsed)) {
                return false;
            }
            *value = parsed;
            return true;
        } catch (const std::exception&) {
            return false;
        }
    }

    bool inputFresh() const
    {
        return has_latest_input_ &&
            (ros::WallTime::now() - latest_input_time_).toSec() <=
                max_input_age_s_;
    }

    bool valueAllowed(double value) const
    {
        return std::isfinite(value) && std::abs(value) <= max_angle_rad_;
    }

    const char* faultName(Fault fault) const
    {
        switch (fault) {
        case Fault::OFFSET: return "STEEROFFSET_command";
        case Fault::FREEZE: return "STEERFREEZE_command";
        case Fault::SATURATION: return "STEERSAT_command";
        case Fault::OSCILLATION: return "STEEROSC_command";
        case Fault::NONE: return "steering_output";
        }
        return "steering_output";
    }

    void publishStatus(const std::string& event, const std::string& fault,
                       bool active, double value, const std::string& detail)
    {
        double remaining_s = 0.0;
        if (enable_fault_timeout_ && state_ == State::FAULT_ACTIVE) {
            remaining_s = std::max(
                0.0, (fault_deadline_ - ros::WallTime::now()).toSec()
            );
        }
        std_msgs::String status;
        std::ostringstream json;
        json << "{\"source\":\"steering\",\"event\":\"" << event
             << "\",\"fault\":\"" << fault << "\",\"active\":"
             << (active ? "true" : "false") << ",\"value\":" << value
             << ",\"timeout_s\":"
             << (enable_fault_timeout_ ? fault_timeout_s_ : 0.0)
             << ",\"remaining_s\":" << remaining_s
             << ",\"detail\":\"" << detail << "\"}";
        status.data = json.str();
        status_pub_.publish(status);
    }

    void publishInhibit(bool inhibit)
    {
        std_msgs::Bool message;
        message.data = inhibit;
        inhibit_pub_.publish(message);
    }

    bool publishOutput(double value)
    {
        if (!valueAllowed(value)) {
            enterInhibited("steering with the fault applied is outside the allowed range");
            return false;
        }
        std_msgs::Float64 output;
        output.data = value;
        output_pub_.publish(output);
        last_published_value_ = value;
        last_published_time_ = ros::WallTime::now();
        has_last_published_ = true;
        return true;
    }

    void enterInhibited(const std::string& reason)
    {
        if (state_ == State::INHIBITED) {
            return;
        }
        const Fault interrupted_fault = active_fault_;
        state_ = State::INHIBITED;
        publishInhibit(true);
        publishStatus("inhibited", faultName(interrupted_fault),
                      interrupted_fault != Fault::NONE, fault_value_, reason);
        ROS_ERROR("Steering output stopped: %s", reason.c_str());
    }

    bool activationReady(const std::string& fault, double requested_value)
    {
        if (state_ != State::PASS_THROUGH || active_fault_ != Fault::NONE) {
            publishStatus("rejected", fault, false, requested_value,
                          "another steering fault or recovery is active");
            return false;
        }
        // DISABLED FOR CURRENT LIVE TESTS:
        // A valid command must exist, but it can be older than 0.3 seconds.
        if (!has_latest_input_ || !has_last_published_ ||
            !valueAllowed(last_published_value_)) {
            publishStatus("rejected", fault, false, requested_value,
                          "no valid pre-click steering output");
            return false;
        }
        return true;
    }

    void activateFault(Fault fault, double value)
    {
        active_fault_ = fault;
        fault_value_ = value;
        frozen_value_ = last_published_value_;
        fault_started_ = ros::WallTime::now();
        if (enable_fault_timeout_) {
            fault_deadline_ = fault_started_ + ros::WallDuration(fault_timeout_s_);
        }
        state_ = State::FAULT_ACTIVE;
        publishStatus("accepted", faultName(fault), true, value,
                      "physical steering fault activated");
    }

    void beginRecovery(const std::string& reason)
    {
        // DISABLED FOR CURRENT LIVE TESTS:
        // Do not stop recovery only because the last command is over 0.3 s old.
        if (!has_latest_input_ || !valueAllowed(latest_input_)) {
            enterInhibited("cannot recover: steering source missing or invalid");
            return;
        }
        recovery_value_ = has_last_published_ ? last_published_value_ : latest_input_;
        state_ = State::RECOVER;
        publishStatus("recovery", faultName(active_fault_), true,
                      fault_value_, reason);
    }

    void deactivateFault(Fault fault)
    {
        if (state_ == State::INHIBITED && active_fault_ == fault &&
            has_latest_input_ && valueAllowed(latest_input_)) {
            const std::string completed_fault = faultName(active_fault_);
            active_fault_ = Fault::NONE;
            fault_value_ = 0.0;
            state_ = State::PASS_THROUGH;
            publishInhibit(false);
            publishOutput(latest_input_);
            publishStatus("reset", completed_fault, false, latest_input_,
                          "fault reset; using the current input again");
            return;
        }
        if (active_fault_ != fault ||
            (state_ != State::FAULT_ACTIVE && state_ != State::RECOVER)) {
            publishStatus("reset", faultName(fault), false, 0.0,
                          "fault already inactive");
            return;
        }
        beginRecovery("manual fault release");
    }

    double faultedOutput(const ros::WallTime& now) const
    {
        double value = latest_input_;
        switch (active_fault_) {
        case Fault::OFFSET:
            value += fault_value_;
            break;
        case Fault::FREEZE:
            value = frozen_value_;
            break;
        case Fault::SATURATION:
            value = std::max(-fault_value_, std::min(value, fault_value_));
            break;
        case Fault::OSCILLATION: {
            const double two_pi = 6.28318530717958647692;
            const double elapsed_s = (now - fault_started_).toSec();
            value += fault_value_ *
                std::sin(two_pi * oscillation_frequency_hz_ * elapsed_s);
            break;
        }
        case Fault::NONE:
            break;
        }
        return std::max(-max_angle_rad_, std::min(value, max_angle_rad_));
    }

    void inputCallback(const std_msgs::Float64::ConstPtr& message)
    {
        if (!valueAllowed(message->data)) {
            enterInhibited("nominal steering outside configured bounds");
            return;
        }
        latest_input_ = message->data;
        latest_input_time_ = ros::WallTime::now();
        has_latest_input_ = true;
        source_ended_ = false;

        if (state_ == State::INHIBITED && active_fault_ == Fault::NONE) {
            state_ = State::PASS_THROUGH;
            publishInhibit(false);
            publishOutput(latest_input_);
            publishStatus("reset", "steering_output", false, latest_input_,
                          "steering input is available again");
        } else if (state_ == State::PASS_THROUGH) {
            publishOutput(latest_input_);
        }
    }

    void handleToggle(Fault fault, const std::string& command, double value)
    {
        if (value == 0.0) {
            deactivateFault(fault);
        } else if (value != 1.0) {
            publishStatus("rejected", command, false, value,
                          "toggle fault accepts only 0 or 1");
        } else if (activationReady(command, value)) {
            activateFault(fault, value);
        }
    }

    void handleValue(Fault fault, const std::string& command, double value,
                     double minimum, double maximum)
    {
        if (value == 0.0) {
            deactivateFault(fault);
        } else if (value < minimum || value > maximum) {
            publishStatus("rejected", command, false, value,
                          "fault value is outside the test limit");
        } else if (activationReady(command, value)) {
            activateFault(fault, value);
        }
    }

    void commandCallback(const std_msgs::String::ConstPtr& message)
    {
        double value = 0.0;
        if (parseValue(message->data, "STEEROFFSET_command=", &value)) {
            handleValue(Fault::OFFSET, "STEEROFFSET_command", value,
                        -max_offset_rad_, max_offset_rad_);
        } else if (parseValue(message->data, "STEERFREEZE_command=", &value)) {
            handleToggle(Fault::FREEZE, "STEERFREEZE_command", value);
        } else if (parseValue(message->data, "STEERSAT_command=", &value)) {
            handleValue(Fault::SATURATION, "STEERSAT_command", value,
                        0.0, max_saturation_rad_);
        } else if (parseValue(message->data, "STEEROSC_command=", &value)) {
            handleValue(Fault::OSCILLATION, "STEEROSC_command", value,
                        0.0, max_oscillation_rad_);
        } else if (parseValue(message->data, "STEERRANDOM_command=", &value)) {
            publishStatus("rejected", "STEERRANDOM_command", false, value,
                          "random steering is simulation-only");
        }
    }

    void configCallback(const std_msgs::String::ConstPtr& message)
    {
        double value = 0.0;
        const std::string command = "PHYSICAL_FAULT_TIMEOUT_command";
        if (!parseValue(message->data, command + "=", &value)) {
            return;
        }
        if (!enable_fault_timeout_) {
            publishStatus("rejected", command, false, value,
                          "physical fault timeout is disabled");
        } else if (state_ == State::FAULT_ACTIVE || state_ == State::RECOVER) {
            publishStatus("rejected", command, false, value,
                          "cannot change timeout while a steering fault is active");
        } else if (value <= 0.0 || value > max_fault_timeout_s_) {
            publishStatus("rejected", command, false, value,
                          "timeout outside configured limits");
        } else {
            fault_timeout_s_ = value;
            publishStatus("configured", command, false, value,
                          "steering fault timeout updated");
        }
    }

    void timerCallback(const ros::WallTimerEvent&)
    {
        const ros::WallTime now = ros::WallTime::now();
        const double elapsed_s = std::max(
            0.0, (now - last_timer_time_).toSec()
        );
        last_timer_time_ = now;

        if (zero_on_source_loss_ && has_latest_input_ &&
            (now - latest_input_time_).toSec() > source_loss_timeout_s_) {
            if (!source_ended_) {
                const std::string ended_fault = faultName(active_fault_);
                active_fault_ = Fault::NONE;
                fault_value_ = 0.0;
                state_ = State::PASS_THROUGH;
                source_ended_ = true;
                ROS_WARN("Recorded steering source ended; publishing zero steering");
                publishStatus("reset", ended_fault, false, 0.0,
                              "recorded steering ended; sending zero");
            }
            publishOutput(0.0);
            return;
        }

        if (state_ == State::PASS_THROUGH) {
            // DISABLED FOR CURRENT LIVE TESTS:
            // if (has_latest_input_ && !inputFresh()) {
            //     enterInhibited("nominal steering source stale");
            // }
            return;
        }
        if (state_ == State::FAULT_ACTIVE) {
            // DISABLED FOR CURRENT LIVE TESTS:
            // if (!inputFresh()) {
            //     enterInhibited("steering source became stale during fault");
            //     return;
            // }
            if (enable_fault_timeout_ && now >= fault_deadline_) {
                publishStatus("timeout", faultName(active_fault_), true,
                              fault_value_, "fault timeout expired");
                beginRecovery("fault time ended; returning gradually to the recorded value");
            } else if (publishOutput(faultedOutput(now))) {
                publishStatus("active", faultName(active_fault_), true,
                              fault_value_, "sending steering with the fault applied");
            }
            return;
        }
        if (state_ == State::RECOVER) {
            // DISABLED FOR CURRENT LIVE TESTS:
            // if (!inputFresh()) {
            //     enterInhibited("steering source stale during recovery");
            //     return;
            // }
            const double maximum_step = max_recovery_rate_rad_s_ * elapsed_s;
            const double delta = latest_input_ - recovery_value_;
            if (std::abs(delta) <= maximum_step) {
                recovery_value_ = latest_input_;
                if (publishOutput(recovery_value_)) {
                    const std::string completed_fault = faultName(active_fault_);
                    active_fault_ = Fault::NONE;
                    fault_value_ = 0.0;
                    state_ = State::PASS_THROUGH;
                    publishStatus("reset", completed_fault, false,
                                  recovery_value_, "normal steering commands restored");
                }
            } else {
                recovery_value_ += std::max(-maximum_step,
                                            std::min(delta, maximum_step));
                publishOutput(recovery_value_);
            }
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher output_pub_;
    ros::Publisher status_pub_;
    ros::Publisher inhibit_pub_;
    ros::Subscriber input_sub_;
    ros::Subscriber command_sub_;
    ros::Subscriber config_sub_;
    ros::WallTimer timer_;
    std::string input_topic_;
    std::string output_topic_;
    double publish_rate_hz_ = 20.0;
    double fault_timeout_s_ = 5.0;
    double max_fault_timeout_s_ = 300.0;
    double max_input_age_s_ = 0.3;
    double max_angle_rad_ = 0.85;
    double max_offset_rad_ = 0.017453292519943295;
    double max_saturation_rad_ = 0.17453292519943295;
    double max_oscillation_rad_ = 0.008726646259971648;
    double oscillation_frequency_hz_ = 1.0;
    double max_recovery_rate_rad_s_ = 0.2;
    bool valid_configuration_ = true;
    bool enable_fault_injection_ = true;
    bool enable_fault_timeout_ = false;
    bool zero_on_source_loss_ = false;
    bool source_ended_ = false;
    double source_loss_timeout_s_ = 2.0;
    State state_ = State::PASS_THROUGH;
    Fault active_fault_ = Fault::NONE;
    bool has_latest_input_ = false;
    double latest_input_ = 0.0;
    ros::WallTime latest_input_time_;
    bool has_last_published_ = false;
    double last_published_value_ = 0.0;
    ros::WallTime last_published_time_;
    double fault_value_ = 0.0;
    double frozen_value_ = 0.0;
    double recovery_value_ = 0.0;
    ros::WallTime fault_started_;
    ros::WallTime fault_deadline_;
    ros::WallTime last_timer_time_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "steering_fault_proxy");
    SteeringFaultProxy proxy;
    if (!proxy.validConfiguration()) {
        return 1;
    }
    ros::spin();
    return 0;
}
