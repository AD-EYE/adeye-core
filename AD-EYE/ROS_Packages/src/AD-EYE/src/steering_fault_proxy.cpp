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
        private_nh_.param("max_steering_angle_rad", max_angle_rad_, 35.0 * M_PI / 180.0);
        private_nh_.param("max_steering_offset_rad", max_offset_rad_,
                          0.017453292519943295);
        private_nh_.param("min_steering_saturation_rad", min_saturation_rad_,
                          0.017453292519943295);
        private_nh_.param("max_steering_saturation_rad", max_saturation_rad_,
                          0.17453292519943295);
        private_nh_.param("min_steering_oscillation_rad", min_oscillation_rad_,
                          0.0008726646259971648);
        private_nh_.param("max_steering_oscillation_rad", max_oscillation_rad_,
                          0.008726646259971648);
        offset_min_rad_ = -max_offset_rad_;
        offset_max_rad_ = max_offset_rad_;
        saturation_min_rad_ = min_saturation_rad_;
        saturation_max_rad_ = max_saturation_rad_;
        oscillation_min_rad_ = min_oscillation_rad_;
        oscillation_max_rad_ = max_oscillation_rad_;
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
    enum class Fault { NONE, OFFSET, FREEZE, SATURATION, OSCILLATION, CONSTANT };

    bool parametersValid() const
    {
        return std::isfinite(publish_rate_hz_) && publish_rate_hz_ > 0.0 &&
            std::isfinite(fault_timeout_s_) && fault_timeout_s_ > 0.0 &&
            std::isfinite(max_fault_timeout_s_) && max_fault_timeout_s_ > 0.0 &&
            fault_timeout_s_ <= max_fault_timeout_s_ &&
            std::isfinite(max_input_age_s_) && max_input_age_s_ > 0.0 &&
            std::isfinite(max_angle_rad_) && max_angle_rad_ > 0.0 &&
            std::isfinite(max_offset_rad_) && max_offset_rad_ > 0.0 &&
            std::isfinite(min_saturation_rad_) && min_saturation_rad_ > 0.0 &&
            std::isfinite(max_saturation_rad_) && max_saturation_rad_ > 0.0 &&
            min_saturation_rad_ <= max_saturation_rad_ &&
            max_saturation_rad_ <= max_angle_rad_ &&
            std::isfinite(min_oscillation_rad_) && min_oscillation_rad_ > 0.0 &&
            std::isfinite(max_oscillation_rad_) && max_oscillation_rad_ > 0.0 &&
            min_oscillation_rad_ <= max_oscillation_rad_ &&
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

    bool parseLimits(const std::string& text, std::string* fault,
                     double* minimum, double* maximum) const
    {
        const std::string prefix = "FAULT_LIMITS_command=";
        if (text.compare(0, prefix.size(), prefix) != 0) {
            return false;
        }
        const std::string values = text.substr(prefix.size());
        const std::size_t first_comma = values.find(',');
        const std::size_t second_comma = values.find(',', first_comma + 1);
        if (first_comma == std::string::npos ||
            second_comma == std::string::npos ||
            values.find(',', second_comma + 1) != std::string::npos) {
            return false;
        }
        try {
            std::size_t minimum_used = 0;
            std::size_t maximum_used = 0;
            const std::string minimum_text = values.substr(
                first_comma + 1, second_comma - first_comma - 1
            );
            const std::string maximum_text = values.substr(second_comma + 1);
            const double parsed_minimum = std::stod(minimum_text, &minimum_used);
            const double parsed_maximum = std::stod(maximum_text, &maximum_used);
            if (minimum_used != minimum_text.size() ||
                maximum_used != maximum_text.size() ||
                !std::isfinite(parsed_minimum) ||
                !std::isfinite(parsed_maximum)) {
                return false;
            }
            *fault = values.substr(0, first_comma);
            *minimum = parsed_minimum;
            *maximum = parsed_maximum;
            return !fault->empty();
        } catch (const std::exception&) {
            return false;
        }
    }

    bool parseConstantCommand(const std::string& text, double* value,
                              double* duration_s) const
    {
        const std::string prefix = "STEERCONSTANT_command=";
        if (text.compare(0, prefix.size(), prefix) != 0) {
            return false;
        }
        const std::string values = text.substr(prefix.size());
        const std::size_t comma = values.find(',');
        if (comma == std::string::npos ||
            values.find(',', comma + 1) != std::string::npos) {
            return false;
        }
        try {
            std::size_t value_used = 0;
            std::size_t duration_used = 0;
            const std::string value_text = values.substr(0, comma);
            const std::string duration_text = values.substr(comma + 1);
            const double parsed_value = std::stod(value_text, &value_used);
            const double parsed_duration = std::stod(duration_text, &duration_used);
            if (value_used != value_text.size() ||
                duration_used != duration_text.size() ||
                !std::isfinite(parsed_value) ||
                !std::isfinite(parsed_duration)) {
                return false;
            }
            *value = parsed_value;
            *duration_s = parsed_duration;
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
        case Fault::CONSTANT: return "STEERCONSTANT_command";
        case Fault::NONE: return "steering_output";
        }
        return "steering_output";
    }

    void publishStatus(const std::string& event, const std::string& fault,
                       bool active, double value, const std::string& detail,
                       double minimum = 0.0, double maximum = 0.0,
                       bool include_limits = false,
                       double hard_minimum = 0.0, double hard_maximum = 0.0,
                       bool include_hard_limits = false)
    {
        double remaining_s = 0.0;
        if (active_fault_ == Fault::CONSTANT && constant_duration_s_ > 0.0 &&
            state_ == State::FAULT_ACTIVE) {
            remaining_s = std::max(
                0.0, (constant_deadline_ - ros::WallTime::now()).toSec()
            );
        } else if (enable_fault_timeout_ && state_ == State::FAULT_ACTIVE) {
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
             << (active_fault_ == Fault::CONSTANT
                     ? constant_duration_s_
                     : (enable_fault_timeout_ ? fault_timeout_s_ : 0.0))
             << ",\"remaining_s\":" << remaining_s;
        if (include_limits) {
            json << ",\"min_value\":" << minimum
                 << ",\"max_value\":" << maximum;
        }
        if (include_hard_limits) {
            json << ",\"hard_min_value\":" << hard_minimum
                 << ",\"hard_max_value\":" << hard_maximum;
        }
        json << ",\"detail\":\"" << detail << "\"}";
        status.data = json.str();
        status_pub_.publish(status);
    }

    void publishFaultLimits()
    {
        publishStatus("limits", "STEEROFFSET_command", false, 0.0,
                      "steering offset ranges", offset_min_rad_, offset_max_rad_,
                      true, -max_offset_rad_, max_offset_rad_, true);
        publishStatus("limits", "STEERSAT_command", false, 0.0,
                      "steering saturation ranges",
                      saturation_min_rad_, saturation_max_rad_, true,
                      min_saturation_rad_, max_saturation_rad_, true);
        publishStatus("limits", "STEEROSC_command", false, 0.0,
                      "steering oscillation ranges",
                      oscillation_min_rad_, oscillation_max_rad_, true,
                      min_oscillation_rad_, max_oscillation_rad_, true);
    }

    void publishInhibit(bool inhibit)
    {
        std_msgs::Bool message;
        message.data = inhibit;
        inhibit_pub_.publish(message);
    }

    bool publishOutput(double value)
    {
        // Continuous commands and their return to normal have no range cap for now.
        // Restore valueAllowed(value) here to cap continuous sending too.
        if (!std::isfinite(value) ||
            (active_fault_ != Fault::CONSTANT && !valueAllowed(value))) {
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
        // Turned off for current live tests:
        // At least one steering command is needed, but it may be older than 0.3 seconds.
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

    void activateConstant(double value, double duration_s)
    {
        const std::string fault = "STEERCONSTANT_command";
        if (state_ != State::PASS_THROUGH || active_fault_ != Fault::NONE) {
            publishStatus("rejected", fault, false, value,
                          "another steering fault or recovery is active");
            return;
        }
        // Range check disabled for continuous sending; keep valid-number checks.
        // if (!valueAllowed(value)) {
        if (!std::isfinite(value)) {
            publishStatus("rejected", fault, false, value,
                          "steering value must be a usable number");
            return;
        }
        if (duration_s < 0.0 || duration_s > max_fault_timeout_s_) {
            publishStatus("rejected", fault, false, value,
                          "duration must be 0 to 300 seconds");
            return;
        }

        active_fault_ = Fault::CONSTANT;
        fault_value_ = value;
        constant_duration_s_ = duration_s;
        fault_started_ = ros::WallTime::now();
        if (duration_s > 0.0) {
            constant_deadline_ = fault_started_ + ros::WallDuration(duration_s);
        }
        state_ = State::FAULT_ACTIVE;
        publishInhibit(false);
        publishStatus("accepted", fault, true, value,
                      duration_s > 0.0
                          ? "continuous steering started for the selected duration"
                          : "continuous steering started until stopped");
    }

    void stopConstant(const std::string& reason)
    {
        if (active_fault_ != Fault::CONSTANT ||
            (state_ != State::FAULT_ACTIVE && state_ != State::RECOVER)) {
            publishStatus("reset", "STEERCONSTANT_command", false, 0.0,
                          "continuous steering is already stopped");
            return;
        }
        if (has_latest_input_ && valueAllowed(latest_input_)) {
            beginRecovery(reason);
            return;
        }

        publishOutput(0.0);
        active_fault_ = Fault::NONE;
        fault_value_ = 0.0;
        constant_duration_s_ = 0.0;
        state_ = State::PASS_THROUGH;
        publishStatus("reset", "STEERCONSTANT_command", false, 0.0,
                      reason + "; no normal input was available, so zero was sent");
    }

    void beginRecovery(const std::string& reason)
    {
        // Turned off for current live tests:
        // Keep returning to normal even if no new command arrived in the last 0.3 seconds.
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
        case Fault::CONSTANT:
            value = fault_value_;
            break;
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
        double duration_s = 0.0;
        if (parseConstantCommand(message->data, &value, &duration_s)) {
            activateConstant(value, duration_s);
        } else if (message->data == "STEERCONSTANT_STOP_command=1") {
            stopConstant("continuous steering stopped by the operator");
        } else if (parseValue(message->data, "STEEROFFSET_command=", &value)) {
            handleValue(Fault::OFFSET, "STEEROFFSET_command", value,
                        offset_min_rad_, offset_max_rad_);
        } else if (parseValue(message->data, "STEERFREEZE_command=", &value)) {
            handleToggle(Fault::FREEZE, "STEERFREEZE_command", value);
        } else if (parseValue(message->data, "STEERSAT_command=", &value)) {
            handleValue(Fault::SATURATION, "STEERSAT_command", value,
                        saturation_min_rad_, saturation_max_rad_);
        } else if (parseValue(message->data, "STEEROSC_command=", &value)) {
            handleValue(Fault::OSCILLATION, "STEEROSC_command", value,
                        oscillation_min_rad_, oscillation_max_rad_);
        } else if (parseValue(message->data, "STEERRANDOM_command=", &value)) {
            publishStatus("rejected", "STEERRANDOM_command", false, value,
                          "random steering is simulation-only");
        }
    }

    void configCallback(const std_msgs::String::ConstPtr& message)
    {
        if (message->data == "FAULT_LIMITS_QUERY_command") {
            publishFaultLimits();
            return;
        }

        double value = 0.0;
        const std::string command = "PHYSICAL_FAULT_TIMEOUT_command";
        if (parseValue(message->data, command + "=", &value)) {
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
          return;
        }

        std::string limit_fault;
        double minimum = 0.0;
        double maximum = 0.0;
        if (!parseLimits(message->data, &limit_fault, &minimum, &maximum)) {
            return;
        }
        if (state_ == State::FAULT_ACTIVE || state_ == State::RECOVER) {
            publishStatus("rejected", limit_fault, false, 0.0,
                          "cannot change the range while a fault is active");
            return;
        }

        bool accepted = false;
        if (limit_fault == "STEEROFFSET_command" && minimum < maximum &&
            minimum >= -max_offset_rad_ && maximum <= max_offset_rad_) {
            offset_min_rad_ = minimum;
            offset_max_rad_ = maximum;
            accepted = true;
        } else if (limit_fault == "STEERSAT_command" &&
                   minimum >= min_saturation_rad_ &&
                   minimum < maximum && maximum <= max_saturation_rad_) {
            saturation_min_rad_ = minimum;
            saturation_max_rad_ = maximum;
            accepted = true;
        } else if (limit_fault == "STEEROSC_command" &&
                   minimum >= min_oscillation_rad_ &&
                   minimum < maximum && maximum <= max_oscillation_rad_) {
            oscillation_min_rad_ = minimum;
            oscillation_max_rad_ = maximum;
            accepted = true;
        } else if (limit_fault.compare(0, 5, "STEER") != 0) {
            return;
        }

        if (accepted) {
            publishStatus("configured", limit_fault, false, 0.0,
                          "steering fault range updated",
                          minimum, maximum, true);
        } else {
            publishStatus("rejected", limit_fault, false, 0.0,
                          "requested range is outside the allowed safety range");
        }
    }

    void timerCallback(const ros::WallTimerEvent&)
    {
        const ros::WallTime now = ros::WallTime::now();
        const double elapsed_s = std::max(
            0.0, (now - last_timer_time_).toSec()
        );
        last_timer_time_ = now;

        if (state_ == State::FAULT_ACTIVE && active_fault_ == Fault::CONSTANT) {
            if (constant_duration_s_ > 0.0 && now >= constant_deadline_) {
                publishStatus("timeout", "STEERCONSTANT_command", true,
                              fault_value_, "continuous steering duration finished");
                stopConstant("continuous steering duration finished");
            } else if (publishOutput(fault_value_)) {
                publishStatus("active", "STEERCONSTANT_command", true,
                              fault_value_, "sending the selected steering value continuously");
            }
            return;
        }

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
            // Turned off for current live tests:
            // if (has_latest_input_ && !inputFresh()) {
            //     enterInhibited("nominal steering source stale");
            // }
            return;
        }
        if (state_ == State::FAULT_ACTIVE) {
            // Turned off for current live tests:
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
            // Turned off for current live tests:
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
                    constant_duration_s_ = 0.0;
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
    double max_angle_rad_ = 35.0 * M_PI / 180.0;
    double max_offset_rad_ = 0.017453292519943295;
    double min_saturation_rad_ = 0.017453292519943295;
    double max_saturation_rad_ = 0.17453292519943295;
    double min_oscillation_rad_ = 0.0008726646259971648;
    double max_oscillation_rad_ = 0.008726646259971648;
    double offset_min_rad_ = -0.017453292519943295;
    double offset_max_rad_ = 0.017453292519943295;
    double saturation_min_rad_ = 0.017453292519943295;
    double saturation_max_rad_ = 0.17453292519943295;
    double oscillation_min_rad_ = 0.0008726646259971648;
    double oscillation_max_rad_ = 0.008726646259971648;
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
    double constant_duration_s_ = 0.0;
    ros::WallTime fault_started_;
    ros::WallTime fault_deadline_;
    ros::WallTime constant_deadline_;
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
