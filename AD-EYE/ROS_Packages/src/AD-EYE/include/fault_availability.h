#ifndef ADEYE_FAULT_AVAILABILITY_H
#define ADEYE_FAULT_AVAILABILITY_H
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <functional>
#include <map>
#include <sstream>
#include <vector>
#include <unistd.h>

// Turns fault choices on or off. Does not send steering/acceleration values or cancel a test.
class FaultAvailability {
    struct Entry { bool baseline = true, enabled = true; unsigned revision = 0; std::string owner; };
    std::map<std::string, Entry> entries_;
    std::function<bool()> busy_;
    std::string component_, boot_, group_;
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
    ros::WallTimer timer_;
    static bool token(const std::string& value) {
        return !value.empty() && value.size() <= 64 && value.find_first_not_of("0123456789abcdef") == std::string::npos;
    }
    void publish(const std::string& request = "", bool accepted = false, const std::string& detail = "snapshot") {
        std_msgs::String message;
        std::ostringstream out;
        out << "{\"schema_version\":1,\"component\":\"" << component_ << "\",\"boot_id\":\"" << boot_
            << "\",\"busy\":" << (busy_() ? "true" : "false") << ",\"request_id\":\"" << request
            << "\",\"accepted\":" << (accepted ? "true" : "false") << ",\"detail\":\"" << detail << "\",\"commands\":{";
        bool first = true;
        for (const auto& pair : entries_) {
            if (!first) out << ',';
            first = false;
            const auto& entry = pair.second;
            out << '"' << pair.first << "\":{\"baseline\":" << (entry.baseline ? "true" : "false")
                << ",\"enabled\":" << (entry.enabled ? "true" : "false") << ",\"revision\":" << entry.revision
                << ",\"owner\":\"" << entry.owner << "\"}";
        }
        out << "}}"; message.data = out.str(); publisher_.publish(message);
    }
    void control(const std_msgs::String::ConstPtr& message) {
        std::string request;
        try {
            boost::property_tree::ptree data;
            std::istringstream input(message->data); boost::property_tree::read_json(input, data);
            if (data.get<int>("schema_version") != 1 || data.get<std::string>("component") != component_) return;
            request = data.get<std::string>("request_id");
            const auto owner = data.get<std::string>("owner");
            if (!token(request) || !token(owner)) return;
            const auto command = data.get<std::string>("fault");
            auto found = entries_.find(command);
            if (found == entries_.end()) { publish(request, false, "unsupported fault"); return; }
            auto& entry = found->second;
            if (data.get<std::string>("boot_id") != boot_ || data.get<unsigned>("revision") != entry.revision) {
                publish(request, false, "component restarted or availability revision changed"); return;
            }
            if (busy_()) { publish(request, false, "controller busy; stop its test or recovery first"); return; }
            if (!entry.owner.empty() && entry.owner != owner) { publish(request, false, "override belongs to another configuration session"); return; }
            const auto operation = data.get<std::string>("operation");
            if (operation == "reset") { entry.enabled = entry.baseline; entry.owner.clear(); }
            else if (operation == "set") {
                const auto value = data.get<std::string>("value");
                if (value != "true" && value != "false") { publish(request, false, "boolean value required"); return; }
                entry.enabled = value == "true";
                entry.owner = entry.enabled == entry.baseline ? "" : owner;
            } else { publish(request, false, "unsupported operation"); return; }
            ++entry.revision;
            publish(request, true, "availability confirmed; no fault or command activated");
        } catch (const std::exception&) {
            if (token(request)) publish(request, false, "invalid availability request");
        }
    }
public:
    FaultAvailability(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                      const std::vector<std::string>& commands, std::function<bool()> busy,
                      const std::string& group, bool group_baseline) : busy_(busy), group_(group) {
        component_ = ros::this_node::getName();
        boot_ = std::to_string(getpid()) + "-" + std::to_string(ros::WallTime::now().toNSec());
        for (const auto& command : commands) {
            Entry entry; private_nh.param("fault_availability/" + command, entry.baseline, true);
            entry.enabled = entry.baseline; entries_[command] = entry;
        }
        Entry parent; parent.baseline = parent.enabled = group_baseline; entries_[group_] = parent;
        publisher_ = nh.advertise<std_msgs::String>("/adeye/fault_availability_state", 10, true);
        subscriber_ = nh.subscribe("/adeye/fault_availability_control", 20, &FaultAvailability::control, this);
        timer_ = nh.createWallTimer(ros::WallDuration(1.0), [this](const ros::WallTimerEvent&) { publish(); });
        publish();
    }
    std::string blockedActivation(const std::string& text) const {
        const auto equal = text.find('=');
        if (equal == std::string::npos) return "";
        const auto command = text.substr(0, equal);
        auto found = entries_.find(command);
        if (found == entries_.end() || command == group_) return "";
        if (found->second.enabled && entries_.at(group_).enabled) return "";
        // Zero stops an ordinary fault, but starts a zero-valued continuous send.
        if (command.find("CONSTANT_command") == std::string::npos) {
            try { size_t used; const auto number = std::stod(text.substr(equal + 1), &used);
                if (number == 0.0 && used == text.size() - equal - 1) return "";
            } catch (...) {}
        }
        return command;
    }
};
#endif
