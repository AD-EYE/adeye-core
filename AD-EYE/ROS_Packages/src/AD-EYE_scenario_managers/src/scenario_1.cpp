#include "ros/ros.h"
#include <scenario_manager_template.h>


#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <sstream>
#include <boost/filesystem.hpp>

/*!
 * \brief Scenario1 node : Manages start and stop of the recording & experiment for Scenario1
 */
class Scenario1: public ScenarioManagerTemplate {

    private:
        ros::Subscriber speed_sub_;
        float SPEED_THRESHOLD_STOP_ = 0.0f;
        float ego_speed_ = 0.0;

        void speedCallback(geometry_msgs::TwistStamped msg)
        {
            ego_speed_ = msg.twist.linear.x;
        }

    public:
        /*!
        * \brief Constructor
        * \param nh ros::NodeHandle initialized in the main function.
        * \param frequency The frequency at which the main loop will be run.
        * \details Initializes the node and its components such the as subscribers.
        */
        Scenario1(ros::NodeHandle nh, int frequency): ScenarioManagerTemplate(nh, frequency)
        {
            speed_sub_ = ScenarioManagerTemplate::nh_.subscribe("/current_velocity", 10, &Scenario1::speedCallback, this);
            // ScenarioManagerTemplate::nh_.param<float>("/simulink/rain_intensity", rain_intensity_, 0.0);
        }

        

        /*!
        * \brief Starts the experiment, contains what needs to be done during the experiment
        */
        void startExperiment()
        {
            std::cout << "Scenario1: started experiment" << std::endl;
        }

        /*!
        * \brief Stops the experiment, contains the post experiment cleanups
        */
        void stopExperiment()
        {
            std::cout << "Scenario1: stopped experiment" << std::endl;
        }

        /*!
        * \brief Starts the experiment relevant recording
        */
        void startRecording()
        {
            std::cout << "Scenario1: started recording" << std::endl;
        }

        /*!
        * \brief Stops the the experiment relevant recording
        */
        void stopRecording()
        {
            std::cout << "Scenario1: stopped recording" << std::endl;
        }






        /*!
        * \brief Checks if the conditions to start recording are met
        */
        bool startRecordingConditionFulfilled()
        {
            return (ego_speed_ > 2);
        }

        /*!
        * \brief Checks if the conditions to stop recording are met
        */
        bool stopRecordingConditionFulfilled()
        {
            
            return (ego_speed_ < 2);
        }

        /*!
        * \brief Checks if the conditions to start the experiment are met
        */
        bool startExperimentConditionFulfilled()
        {
            return (ego_speed_ > 1);
        }

        /*!
        * \brief Checks if the conditions to stop the experiment are met
        */
        bool stopExperimentConditionFulfilled()
        {
            
            return (ego_speed_ < 1);
        }

        


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Scenario1");
    ros::NodeHandle private_nh("~");

    std::cout << "Creating Scenario1" << std::endl;
    Scenario1 scenario_1(private_nh, 20);
    scenario_1.run();
}