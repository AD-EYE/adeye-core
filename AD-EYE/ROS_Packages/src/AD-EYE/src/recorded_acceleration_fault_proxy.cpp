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
        private_nh_.param("min_acceleration", min_acceleration_, -0.5);
        private_nh_.param("max_acceleration", max_acceleration_, 0.75);
        private_nh_.param("max_acceleration_offset", max_offset_, 0.2);
        private_nh_.param("min_acceleration_saturation", min_saturation_, 0.1);
        private_nh_.param("max_acceleration_saturation", max_saturation_, 0.5);
        private_nh_.param("min_acceleration_oscillation", min_oscillation_, 0.01);
        private_nh_.param("max_acceleration_oscillation", max_oscillation_, 0.1);
        offset_min_ = -max_offset_;
        offset_max_ = max_offset_;
        saturation_min_ = min_saturation_;
        saturation_max_ = max_saturation_;
        oscillation_min_ = min_oscillation_;
        oscillation_max_ = max_oscillation_;
        private_nh_.param("oscillation_frequency_hz", oscillation_frequency_hz_,
                          1.0);
        private_nh_.param("max_recovery_jerk", max_recovery_jerk_, 1.0);
        private_nh_.param("enable_fault_injection", enable_fault_injection_, true);
        private_nh_.param("enable_fault_timeout", enable_fault_timeout_, false);
        private_nh_.param("zero_on_source_loss", zero_on_source_loss_, false);
        private_nh_.param("source_loss_timeout_s", source_loss_timeout_s_, 2.0);

        if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0 ||
            !std::isfinite(fault_timeout_s_) || fault_timeout_s_ <= 0.0 ||
            !std::isfinite(max_fault_timeout_s_) || max_fault_timeout_s_ <= 0.0 ||
            fault_timeout_s_ > max_fault_timeout_s_ ||
            !std::isfinite(max_input_age_s_) || max_input_age_s_ <= 0.0 ||
            !std::isfinite(min_acceleration_) || !std::isfinite(max_acceleration_) ||
            min_acceleration_ >= max_acceleration_ ||
            !std::isfinite(max_offset_) || max_offset_ <= 0.0 ||
            !std::isfinite(min_saturation_) || min_saturation_ <= 0.0 ||
            !std::isfinite(max_saturation_) || max_saturation_ <= 0.0 ||
            min_saturation_ > max_saturation_ ||
            !std::isfinite(min_oscillation_) || min_oscillation_ <= 0.0 ||
            !std::isfinite(max_oscillation_) || max_oscillation_ <= 0.0 ||
            min_oscillation_ > max_oscillation_ ||
            !std::isfinite(oscillation_frequency_hz_) ||
            oscillation_frequency_hz_ <= 0.0 ||
            !std::isfinite(max_recovery_jerk_) || max_recovery_jerk_ <= 0.0 ||
            !std::isfinite(source_loss_timeout_s_) ||
            source_loss_timeout_s_ <= 0.0) {
            ROS_FATAL("Acceleration proxy settings are not valid");
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
        if (enable_fault_injection_) {
            command_sub_ = nh_.subscribe<std_msgs::String>(
                "/vehicle_commands", 10,
                &RecordedAccelerationFaultProxy::commandCallback, this
            );
            config_sub_ = nh_.subscribe<std_msgs::String>(
                "/adeye/physical_fault_config", 10,
                &RecordedAccelerationFaultProxy::configCallback, this
            );
        }

        const ros::WallDuration period(1.0 / publish_rate_hz_);
        timer_ = nh_.createWallTimer(
            period, &RecordedAccelerationFaultProxy::timerCallback, this
        );
        last_timer_time_ = ros::WallTime::now();

        publishInhibit(false);
        ROS_INFO("Recorded acceleration proxy: %s -> %s at %.3f Hz",
                 input_topic_.c_str(), output_topic_.c_str(), publish_rate_hz_);
        ROS_INFO("Acceleration fault injection: %s",
                 enable_fault_injection_ ? "enabled" : "disabled (normal commands only)");
    }

    bool validConfiguration() const { return valid_configuration_; }

private:
    enum class State { PASS_THROUGH, FAULT_ACTIVE, RECOVER, INHIBITED };
    enum class Fault { NONE, OFFSET, FREEZE, SATURATION, OSCILLATION, CONSTANT };

    const char* faultName(Fault fault) const
    {
        switch (fault) {
        case Fault::OFFSET: return "ACCELOFFSET_command";
        case Fault::FREEZE: return "ACCELFREEZE_command";
        case Fault::SATURATION: return "ACCELSAT_command";
        case Fault::OSCILLATION: return "ACCELOSC_command";
        case Fault::CONSTANT: return "ACCELCONSTANT_command";
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
        const std::string prefix = "ACCELCONSTANT_command=";
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
            (ros::WallTime::now() - latest_input_time_).toSec() <= max_input_age_s_;
    }

    bool valueAllowed(double value) const
    {
        return std::isfinite(value) && value >= min_acceleration_ &&
            value <= max_acceleration_;
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
        json << "{\"source\":\"acceleration\",\"event\":\"" << event
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
        publishStatus("limits", "ACCELOFFSET_command", false, 0.0,
                      "acceleration offset ranges", offset_min_, offset_max_,
                      true, -max_offset_, max_offset_, true);
        publishStatus("limits", "ACCELSAT_command", false, 0.0,
                      "acceleration saturation ranges",
                      saturation_min_, saturation_max_, true,
                      min_saturation_, max_saturation_, true);
        publishStatus("limits", "ACCELOSC_command", false, 0.0,
                      "acceleration oscillation ranges",
                      oscillation_min_, oscillation_max_, true,
                      min_oscillation_, max_oscillation_, true);
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
            enterInhibited("acceleration with the fault applied is outside the allowed range");
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

    bool publishRecordedOutput(double value)
    {
        if (!std::isfinite(value)) {
            ROS_ERROR("Recorded acceleration rejected: value is not a usable number");
            return false;
        }
        if (!valueAllowed(value)) {
            ROS_WARN_THROTTLE(1.0,
                              "Recorded acceleration %.9f is outside configured range [%.9f, %.9f]; publishing it unchanged",
                              value, min_acceleration_, max_acceleration_);
        }
        std_msgs::Float64 output;
        output.data = value;
        output_pub_.publish(output);
        last_published_value_ = value;
        last_published_time_ = ros::WallTime::now();
        has_last_published_ = true;
        return true;
    }

    bool publishFaultedOutput(double value)
    {
        if (valueAllowed(value)) {
            return publishOutput(value);
        }

        const std::string stopped_fault = faultName(active_fault_);
        std::ostringstream detail;
        detail << "fault stopped: calculated output " << value
               << " exceeded configured range [" << min_acceleration_
               << ", " << max_acceleration_
               << "]; original replay value restored";
        ROS_ERROR("%s", detail.str().c_str());

        active_fault_ = Fault::NONE;
        fault_value_ = 0.0;
        state_ = State::PASS_THROUGH;
        publishStatus("rejected", stopped_fault, false, value, detail.str());
        publishRecordedOutput(latest_input_);
        return false;
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
        ROS_ERROR("Acceleration output stopped: %s", reason.c_str());
    }

    void beginRecovery(const std::string& reason)
    {
        // Turned off for current live tests:
        // Keep returning to normal even if no new command arrived in the last 0.3 seconds.
        if (!has_latest_input_ || !std::isfinite(latest_input_)) {
            enterInhibited("cannot recover: acceleration source missing or invalid");
            return;
        }
        if (!valueAllowed(latest_input_)) {
            const std::string completed_fault = faultName(active_fault_);
            active_fault_ = Fault::NONE;
            fault_value_ = 0.0;
            state_ = State::PASS_THROUGH;
            publishRecordedOutput(latest_input_);
            publishStatus("reset", completed_fault, false, latest_input_,
                          "fault ended; using the recorded value again");
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
        // Turned off for current live tests:
        // At least one acceleration command is needed, but it may be older than 0.3 seconds.
        if (!has_latest_input_ || !std::isfinite(latest_input_) ||
            !has_last_published_ || !std::isfinite(last_published_value_)) {
            publishStatus("rejected", fault, false, requested_value,
                          "no valid pre-click output");
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
                      "physical acceleration fault activated");
    }

    void activateConstant(double value, double duration_s)
    {
        const std::string fault = "ACCELCONSTANT_command";
        if (state_ != State::PASS_THROUGH || active_fault_ != Fault::NONE) {
            publishStatus("rejected", fault, false, value,
                          "another acceleration fault or recovery is active");
            return;
        }
        // Range check disabled for continuous sending; keep valid-number checks.
        // if (!valueAllowed(value)) {
        if (!std::isfinite(value)) {
            publishStatus("rejected", fault, false, value,
                          "acceleration value must be a usable number");
            return;
        }
        if (duration_s < 0.0 || duration_s > max_fault_timeout_s_) {
            publishStatus("rejected", fault, false, value,
                          "duration is outside the allowed range");
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
                          ? "continuous acceleration started for the selected duration"
                          : "continuous acceleration started until stopped");
    }

    void stopConstant(const std::string& reason)
    {
        if (active_fault_ != Fault::CONSTANT ||
            (state_ != State::FAULT_ACTIVE && state_ != State::RECOVER)) {
            publishStatus("reset", "ACCELCONSTANT_command", false, 0.0,
                          "continuous acceleration is already stopped");
            return;
        }
        if (has_latest_input_ && std::isfinite(latest_input_)) {
            beginRecovery(reason);
            return;
        }

        publishOutput(0.0);
        active_fault_ = Fault::NONE;
        fault_value_ = 0.0;
        constant_duration_s_ = 0.0;
        state_ = State::PASS_THROUGH;
        publishStatus("reset", "ACCELCONSTANT_command", false, 0.0,
                      reason + "; no normal input was available, so zero was sent");
    }

    void deactivateFault(Fault fault, const std::string& reason)
    {
        if (state_ == State::INHIBITED && active_fault_ == fault &&
            has_latest_input_ && std::isfinite(latest_input_)) {
            const std::string completed_fault = faultName(active_fault_);
            active_fault_ = Fault::NONE;
            fault_value_ = 0.0;
            state_ = State::PASS_THROUGH;
            publishInhibit(false);
            publishRecordedOutput(latest_input_);
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
            // Use the same limit for speeding up and slowing down.
            return std::max(-fault_value_,
                            std::min(latest_input_, fault_value_));
        case Fault::OSCILLATION: {
            const double two_pi = 6.28318530717958647692;
            const double elapsed_s = (now - fault_started_).toSec();
            return latest_input_ + fault_value_ *
                std::sin(two_pi * oscillation_frequency_hz_ * elapsed_s);
        }
        case Fault::CONSTANT:
            return fault_value_;
        case Fault::NONE:
            return latest_input_;
        }
        return latest_input_;
    }

    void inputCallback(const std_msgs::Float64::ConstPtr& message)
    {
        if (!std::isfinite(message->data)) {
            ROS_ERROR("Recorded acceleration rejected: input is not a usable number");
            return;
        }
        latest_input_ = message->data;
        latest_input_time_ = ros::WallTime::now();
        has_latest_input_ = true;
        source_ended_ = false;

        if (state_ == State::INHIBITED && active_fault_ == Fault::NONE) {
            state_ = State::PASS_THROUGH;
            publishInhibit(false);
            publishRecordedOutput(latest_input_);
            publishStatus("reset", "acceleration_output", false, latest_input_,
                          "acceleration input is available again");
            return;
        }

        if (state_ == State::PASS_THROUGH) {
            publishRecordedOutput(latest_input_);
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
        } else if (message->data == "ACCELCONSTANT_STOP_command=1") {
            stopConstant("continuous acceleration stopped by the operator");
        } else if (parseValue(message->data, "ACCELFREEZE_command=", &value)) {
            handleToggleFault("ACCELFREEZE_command", Fault::FREEZE, value);
        } else if (parseValue(message->data, "ACCELOFFSET_command=", &value)) {
            handleValueFault("ACCELOFFSET_command", Fault::OFFSET, value,
                             offset_min_, offset_max_);
        } else if (parseValue(message->data, "ACCELSAT_command=", &value)) {
            handleValueFault("ACCELSAT_command", Fault::SATURATION, value,
                             saturation_min_, saturation_max_);
        } else if (parseValue(message->data, "ACCELOSC_command=", &value)) {
            handleValueFault("ACCELOSC_command", Fault::OSCILLATION, value,
                             oscillation_min_, oscillation_max_);
        } else if (parseValue(message->data, "ACCELRUNAWAY_command=", &value)) {
            publishStatus("rejected", "ACCELRUNAWAY_command", false, value,
                          "runaway acceleration is simulation-only");
        }
    }

    void configCallback(const std_msgs::String::ConstPtr& message)
    {
        if (message->data == "FAULT_LIMITS_QUERY_command") {
            publishFaultLimits();
            return;
        }

        double value = 0.0;
        const std::string fault = "PHYSICAL_FAULT_TIMEOUT_command";
        if (parseValue(message->data, fault + "=", &value)) {
          if (!enable_fault_timeout_) {
            publishStatus("rejected", fault, false, value,
                          "physical fault timeout is disabled");
          } else if (state_ == State::FAULT_ACTIVE || state_ == State::RECOVER) {
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
        if (limit_fault == "ACCELOFFSET_command" && minimum < maximum &&
            minimum >= -max_offset_ && maximum <= max_offset_) {
            offset_min_ = minimum;
            offset_max_ = maximum;
            accepted = true;
        } else if (limit_fault == "ACCELSAT_command" &&
                   minimum >= min_saturation_ &&
                   minimum < maximum && maximum <= max_saturation_) {
            saturation_min_ = minimum;
            saturation_max_ = maximum;
            accepted = true;
        } else if (limit_fault == "ACCELOSC_command" &&
                   minimum >= min_oscillation_ &&
                   minimum < maximum && maximum <= max_oscillation_) {
            oscillation_min_ = minimum;
            oscillation_max_ = maximum;
            accepted = true;
        } else if (limit_fault.compare(0, 5, "ACCEL") != 0) {
            return;
        }

        if (accepted) {
            publishStatus("configured", limit_fault, false, 0.0,
                          "acceleration fault range updated",
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
                publishStatus("timeout", "ACCELCONSTANT_command", true,
                              fault_value_, "continuous acceleration duration finished");
                stopConstant("continuous acceleration duration finished");
            } else if (publishOutput(fault_value_)) {
                publishStatus("active", "ACCELCONSTANT_command", true,
                              fault_value_, "sending the selected acceleration continuously");
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
                ROS_WARN("Recorded acceleration source ended; publishing zero acceleration");
                publishStatus("reset", ended_fault, false, 0.0,
                              "recorded acceleration ended; sending zero");
            }
            publishOutput(0.0);
            return;
        }

        if (state_ == State::PASS_THROUGH) {
            // Turned off for current live tests:
            // if (has_latest_input_ && !inputFresh()) {
            //     enterInhibited("acceleration source stale");
            // }
            return;
        }

        if (state_ == State::FAULT_ACTIVE) {
            // Turned off for current live tests:
            // if (!inputFresh()) {
            //     enterInhibited("acceleration source became stale during fault");
            //     return;
            // }
            if (enable_fault_timeout_ && now >= fault_deadline_) {
                publishStatus("timeout", faultName(active_fault_), true,
                              fault_value_, "fault timeout expired");
                beginRecovery("fault time ended; returning gradually to the recorded value");
            } else {
                if (publishFaultedOutput(faultedOutput(now))) {
                    publishStatus("active", faultName(active_fault_), true,
                                  fault_value_,
                                  "sending acceleration with the fault applied");
                }
            }
            return;
        }

        if (state_ == State::RECOVER) {
            // Turned off for current live tests:
            // if (!inputFresh()) {
            //     enterInhibited("acceleration source stale during recovery");
            //     return;
            // }
            const double target = latest_input_;
            const double maximum_step = max_recovery_jerk_ * elapsed_s;
            const double delta = target - recovery_value_;
            if (std::abs(delta) <= maximum_step) {
                recovery_value_ = target;
                if (publishOutput(recovery_value_)) {
                    const std::string completed_fault = faultName(active_fault_);
                    active_fault_ = Fault::NONE;
                    fault_value_ = 0.0;
                    constant_duration_s_ = 0.0;
                    state_ = State::PASS_THROUGH;
                    publishStatus("reset", completed_fault, false,
                                  recovery_value_, "normal acceleration commands restored");
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
    double min_acceleration_ = -0.5;
    double max_acceleration_ = 0.75;
    double max_offset_ = 0.2;
    double min_saturation_ = 0.1;
    double max_saturation_ = 0.5;
    double min_oscillation_ = 0.01;
    double max_oscillation_ = 0.1;
    double offset_min_ = -0.2;
    double offset_max_ = 0.2;
    double saturation_min_ = 0.1;
    double saturation_max_ = 0.5;
    double oscillation_min_ = 0.01;
    double oscillation_max_ = 0.1;
    double oscillation_frequency_hz_ = 1.0;
    double max_recovery_jerk_ = 1.0;
    bool valid_configuration_ = true;
    bool enable_fault_timeout_ = false;
    bool enable_fault_injection_ = true;
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
    double frozen_value_ = 0.0;
    double fault_value_ = 0.0;
    double recovery_value_ = 0.0;
    double constant_duration_s_ = 0.0;
    ros::WallTime fault_started_;
    ros::WallTime fault_deadline_;
    ros::WallTime constant_deadline_;
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
