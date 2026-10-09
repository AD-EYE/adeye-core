#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

class RecordedAccelerationFaultProxy
{
public:
    RecordedAccelerationFaultProxy()
        : nh_(), private_nh_("~")
    {
        private_nh_.param("input_topic", input_topic_,
                          std::string("/replay/acceleration_requested_phy"));
        private_nh_.param("output_topic", output_topic_,
                          std::string("/acceleration_requested_phy"));
        private_nh_.param("publish_rate_hz", publish_rate_hz_, 10.0);
        private_nh_.param("fault_timeout_s", fault_timeout_s_, 5.0);
        private_nh_.param("max_fault_timeout_s", max_fault_timeout_s_, 300.0);
        private_nh_.param("max_input_age_s", max_input_age_s_, 0.3);
        private_nh_.param("min_acceleration", min_acceleration_, -15.0);
        private_nh_.param("max_acceleration", max_acceleration_, 16.99);
        private_nh_.param("max_acceleration_offset", max_offset_, 0.2);
        private_nh_.param("max_acceleration_saturation", max_saturation_, 0.5);
        private_nh_.param("max_acceleration_oscillation", max_oscillation_, 0.1);
        private_nh_.param("oscillation_frequency_hz", oscillation_frequency_hz_,
                          1.0);
        private_nh_.param("max_recovery_jerk", max_recovery_jerk_, 1.0);

        if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0 ||
            !std::isfinite(fault_timeout_s_) || fault_timeout_s_ <= 0.0 ||
            !std::isfinite(max_fault_timeout_s_) || max_fault_timeout_s_ <= 0.0 ||
            fault_timeout_s_ > max_fault_timeout_s_ ||
            !std::isfinite(max_input_age_s_) || max_input_age_s_ <= 0.0 ||
            !std::isfinite(min_acceleration_) || !std::isfinite(max_acceleration_) ||
            min_acceleration_ >= max_acceleration_ ||
            !std::isfinite(max_offset_) || max_offset_ <= 0.0 ||
            !std::isfinite(max_saturation_) || max_saturation_ <= 0.0 ||
            !std::isfinite(max_oscillation_) || max_oscillation_ <= 0.0 ||
            !std::isfinite(oscillation_frequency_hz_) ||
            oscillation_frequency_hz_ <= 0.0 ||
            !std::isfinite(max_recovery_jerk_) || max_recovery_jerk_ <= 0.0) {
            ROS_FATAL("Invalid recorded acceleration proxy parameters");
            valid_configuration_ = false;
            return;
        }

        output_pub_ = nh_.advertise<std_msgs::Float64>(output_topic_, 1);
        status_pub_ = nh_.advertise<std_msgs::String>(
            "/adeye/physical_fault_status", 10
        );
        inhibit_pub_ = nh_.advertise<std_msgs::Bool>(
            "/adeye/acceleration_output_inhibit", 1, true
        );
        input_sub_ = nh_.subscribe<std_msgs::Float64>(
            input_topic_, 1, &RecordedAccelerationFaultProxy::inputCallback, this
        );
        command_sub_ = nh_.subscribe<std_msgs::String>(
            "/vehicle_commands", 10,
            &RecordedAccelerationFaultProxy::commandCallback, this
        );
        config_sub_ = nh_.subscribe<std_msgs::String>(
            "/adeye/physical_fault_config", 10,
            &RecordedAccelerationFaultProxy::configCallback, this
        );

        const ros::WallDuration period(1.0 / publish_rate_hz_);
        timer_ = nh_.createWallTimer(
            period, &RecordedAccelerationFaultProxy::timerCallback, this
        );
        last_timer_time_ = ros::WallTime::now();

        publishInhibit(false);
        ROS_INFO("Recorded acceleration proxy: %s -> %s at %.3f Hz",
                 input_topic_.c_str(), output_topic_.c_str(), publish_rate_hz_);
    }

    bool validConfiguration() const { return valid_configuration_; }

private:
    enum class State { PASS_THROUGH, FAULT_ACTIVE, RECOVER, INHIBITED };
    enum class Fault { NONE, OFFSET, FREEZE, SATURATION, OSCILLATION };

    const char* faultName(Fault fault) const
    {
        switch (fault) {
        case Fault::OFFSET: return "ACCELOFFSET_command";
        case Fault::FREEZE: return "ACCELFREEZE_command";
        case Fault::SATURATION: return "ACCELSAT_command";
        case Fault::OSCILLATION: return "ACCELOSC_command";
        case Fault::NONE: return "acceleration_output";
        }
        return "acceleration_output";
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
            (ros::WallTime::now() - latest_input_time_).toSec() <= max_input_age_s_;
    }

    bool valueAllowed(double value) const
    {
        return std::isfinite(value) && value >= min_acceleration_ &&
            value <= max_acceleration_;
    }

    void publishStatus(const std::string& event, const std::string& fault,
                       bool active, double value, const std::string& detail)
    {
        double remaining_s = 0.0;
        if (state_ == State::FAULT_ACTIVE) {
            remaining_s = std::max(
                0.0, (fault_deadline_ - ros::WallTime::now()).toSec()
            );
        }
        std_msgs::String status;
        std::ostringstream json;
        json << "{\"source\":\"acceleration\",\"event\":\"" << event
             << "\",\"fault\":\"" << fault << "\",\"active\":"
             << (active ? "true" : "false") << ",\"value\":" << value
             << ",\"timeout_s\":" << fault_timeout_s_
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
            enterInhibited("faulted output outside configured final bounds");
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
        const bool fault_requires_explicit_reset =
            interrupted_fault != Fault::NONE;
        state_ = State::INHIBITED;
        publishInhibit(true);
        publishStatus(
            "inhibited",
            faultName(interrupted_fault),
            fault_requires_explicit_reset,
            fault_requires_explicit_reset ? fault_value_ : 0.0,
            reason
        );
        ROS_ERROR("Recorded acceleration output inhibited: %s", reason.c_str());
    }

    void beginRecovery(const std::string& reason)
    {
        if (!inputFresh() || !valueAllowed(latest_input_)) {
            enterInhibited("cannot recover: acceleration source stale or invalid");
            return;
        }
        recovery_value_ = has_last_published_ ? last_published_value_ : latest_input_;
        state_ = State::RECOVER;
        publishStatus("recovery", faultName(active_fault_), true,
                      fault_value_, reason);
    }

    bool activationReady(const std::string& fault, double requested_value)
    {
        if (state_ != State::PASS_THROUGH || active_fault_ != Fault::NONE) {
            publishStatus("rejected", fault, false, requested_value,
                          "another acceleration fault or recovery is active");
            return false;
        }
        const bool published_fresh = has_last_published_ &&
            (ros::WallTime::now() - last_published_time_).toSec() <=
                max_input_age_s_;
        if (!inputFresh() || !published_fresh ||
            !valueAllowed(last_published_value_)) {
            publishStatus("rejected", fault, false, requested_value,
                          "no fresh valid pre-click output");
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
        fault_deadline_ = fault_started_ + ros::WallDuration(fault_timeout_s_);
        state_ = State::FAULT_ACTIVE;
        publishStatus("accepted", faultName(fault), true, value,
                      "physical acceleration fault activated");
    }

    void deactivateFault(Fault fault, const std::string& reason)
    {
        if (state_ == State::INHIBITED && active_fault_ == fault && inputFresh()) {
            const std::string completed_fault = faultName(active_fault_);
            active_fault_ = Fault::NONE;
            fault_value_ = 0.0;
            state_ = State::PASS_THROUGH;
            publishInhibit(false);
            publishOutput(latest_input_);
            publishStatus("reset", completed_fault, false, latest_input_,
                          "explicit fresh-source reset");
            return;
        }
        if (active_fault_ != fault ||
            (state_ != State::FAULT_ACTIVE && state_ != State::RECOVER)) {
            publishStatus("reset", faultName(fault), false, 0.0,
                          "fault already inactive");
            return;
        }
        beginRecovery(reason);
    }

    double faultedOutput(const ros::WallTime& now) const
    {
        switch (active_fault_) {
        case Fault::OFFSET:
            return latest_input_ + fault_value_;
        case Fault::FREEZE:
            return frozen_value_;
        case Fault::SATURATION:
            // Limit forward acceleration only; do not change braking commands.
            return std::min(latest_input_, fault_value_);
        case Fault::OSCILLATION: {
            const double two_pi = 6.28318530717958647692;
            const double elapsed_s = (now - fault_started_).toSec();
            return latest_input_ + fault_value_ *
                std::sin(two_pi * oscillation_frequency_hz_ * elapsed_s);
        }
        case Fault::NONE:
            return latest_input_;
        }
        return latest_input_;
    }

    void inputCallback(const std_msgs::Float64::ConstPtr& message)
    {
        if (!valueAllowed(message->data)) {
            enterInhibited("acceleration source outside configured bounds");
            return;
        }
        latest_input_ = message->data;
        latest_input_time_ = ros::WallTime::now();
        has_latest_input_ = true;

        if (state_ == State::INHIBITED && active_fault_ == Fault::NONE) {
            state_ = State::PASS_THROUGH;
            publishInhibit(false);
            publishOutput(latest_input_);
            publishStatus("reset", "acceleration_output", false, latest_input_,
                          "fresh acceleration source restored");
            return;
        }

        if (state_ == State::PASS_THROUGH) {
            publishOutput(latest_input_);
        }
    }

    void handleToggleFault(const std::string& command, Fault fault,
                           double value)
    {
        if (value == 0.0) {
            deactivateFault(fault, "manual fault release");
        } else if (value != 1.0) {
            publishStatus("rejected", command, false, value,
                          "toggle fault accepts only 0 or 1");
        } else if (activationReady(command, value)) {
            activateFault(fault, value);
        }
    }

    void handleValueFault(const std::string& command, Fault fault, double value,
                          double minimum, double maximum)
    {
        if (value == 0.0) {
            deactivateFault(fault, "manual fault release");
            return;
        }
        const bool in_range = value >= minimum && value <= maximum;
        if (!in_range) {
            publishStatus("rejected", command, false, value,
                          "fault value outside configured physical limit");
        } else if (activationReady(command, value)) {
            activateFault(fault, value);
        }
    }

    void commandCallback(const std_msgs::String::ConstPtr& message)
    {
        double value = 0.0;
        if (parseValue(message->data, "ACCELFREEZE_command=", &value)) {
            handleToggleFault("ACCELFREEZE_command", Fault::FREEZE, value);
        } else if (parseValue(message->data, "ACCELOFFSET_command=", &value)) {
            handleValueFault("ACCELOFFSET_command", Fault::OFFSET, value,
                             -max_offset_, max_offset_);
        } else if (parseValue(message->data, "ACCELSAT_command=", &value)) {
            handleValueFault("ACCELSAT_command", Fault::SATURATION, value,
                             0.0, max_saturation_);
        } else if (parseValue(message->data, "ACCELOSC_command=", &value)) {
            handleValueFault("ACCELOSC_command", Fault::OSCILLATION, value,
                             0.0, max_oscillation_);
        } else if (parseValue(message->data, "ACCELRUNAWAY_command=", &value)) {
            publishStatus("rejected", "ACCELRUNAWAY_command", false, value,
                          "runaway acceleration is simulation-only");
        }
    }

    void configCallback(const std_msgs::String::ConstPtr& message)
    {
        double value = 0.0;
        const std::string fault = "PHYSICAL_FAULT_TIMEOUT_command";
        if (!parseValue(message->data, fault + "=", &value)) {
            return;
        }
        if (state_ == State::FAULT_ACTIVE || state_ == State::RECOVER) {
            publishStatus("rejected", fault, false, value,
                          "cannot change timeout while a fault is active");
        } else if (value <= 0.0 || value > max_fault_timeout_s_) {
            publishStatus("rejected", fault, false, value,
                          "timeout outside configured limits");
        } else {
            fault_timeout_s_ = value;
            publishStatus("configured", fault, false, value,
                          "acceleration fault timeout updated");
        }
    }

    void timerCallback(const ros::WallTimerEvent&)
    {
        const ros::WallTime now = ros::WallTime::now();
        const double elapsed_s = std::max(
            0.0, (now - last_timer_time_).toSec()
        );
        last_timer_time_ = now;

        if (state_ == State::PASS_THROUGH) {
            if (has_latest_input_ && !inputFresh()) {
                enterInhibited("acceleration source stale");
            }
            return;
        }

        if (state_ == State::FAULT_ACTIVE) {
            if (!inputFresh()) {
                enterInhibited("acceleration source became stale during fault");
                return;
            }
            if (now >= fault_deadline_) {
                publishStatus("timeout", faultName(active_fault_), true,
                              fault_value_, "fault timeout expired");
                beginRecovery("fault timeout; starting jerk-limited recovery");
            } else {
                if (publishOutput(faultedOutput(now))) {
                    publishStatus("active", faultName(active_fault_), true,
                                  fault_value_,
                                  "publishing faulted acceleration");
                }
            }
            return;
        }

        if (state_ == State::RECOVER) {
            if (!inputFresh()) {
                enterInhibited("acceleration source stale during recovery");
                return;
            }
            const double target = latest_input_;
            const double maximum_step = max_recovery_jerk_ * elapsed_s;
            const double delta = target - recovery_value_;
            if (std::abs(delta) <= maximum_step) {
                recovery_value_ = target;
                if (publishOutput(recovery_value_)) {
                    const std::string completed_fault = faultName(active_fault_);
                    active_fault_ = Fault::NONE;
                    fault_value_ = 0.0;
                    state_ = State::PASS_THROUGH;
                    publishStatus("reset", completed_fault, false,
                                  recovery_value_, "pass-through restored");
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
    double publish_rate_hz_ = 10.0;
    double fault_timeout_s_ = 5.0;
    double max_fault_timeout_s_ = 300.0;
    double max_input_age_s_ = 0.3;
    double min_acceleration_ = -15.0;
    double max_acceleration_ = 16.99;
    double max_offset_ = 0.2;
    double max_saturation_ = 0.5;
    double max_oscillation_ = 0.1;
    double oscillation_frequency_hz_ = 1.0;
    double max_recovery_jerk_ = 1.0;
    bool valid_configuration_ = true;

    State state_ = State::PASS_THROUGH;
    Fault active_fault_ = Fault::NONE;
    bool has_latest_input_ = false;
    double latest_input_ = 0.0;
    ros::WallTime latest_input_time_;
    bool has_last_published_ = false;
    double last_published_value_ = 0.0;
    ros::WallTime last_published_time_;
    double frozen_value_ = 0.0;
    double fault_value_ = 0.0;
    double recovery_value_ = 0.0;
    ros::WallTime fault_started_;
    ros::WallTime fault_deadline_;
    ros::WallTime last_timer_time_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "recorded_acceleration_fault_proxy");
    RecordedAccelerationFaultProxy proxy;
    if (!proxy.validConfiguration()) {
        return 1;
    }
    ros::spin();
    return 0;
}
