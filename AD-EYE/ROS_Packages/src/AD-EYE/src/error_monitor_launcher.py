#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import subprocess
import threading

def read_stream(stream, publisher, error_keywords):
    """
    Reads lines from the given stream (stdout or stderr).
    If a line contains predefined error keywords, it is published via the publisher.
    """
    # Reads each line until the line is empty (using the iter function)
    for line in iter(stream.readline, b''):
        decoded_line = line.decode('utf-8').strip()
        rospy.loginfo(decoded_line)
        for keyword in error_keywords:
            if keyword in decoded_line.lower():
                publisher.publish(decoded_line)
                break  # Publishing once per line is sufficient
    stream.close()

if __name__ == '__main__':
    rospy.init_node('error_monitor', anonymous=True)
    error_pub = rospy.Publisher('/error_logs', String, queue_size=10)

    # The simulation command to be executed can be taken as a parameter.
    # For example, you can modify the command from your launch file here.
    sim_command = rospy.get_param("~sim_command", "roslaunch adeye manager_real_world.launch")
    
    # Keywords to be used for error detection (checked in lowercase)
    error_keywords = ["process has died", "error", "fail"]

    rospy.loginfo("Starting simulation: {}".format(sim_command))
    
    # Start the simulation command as a subprocess.
    process = subprocess.Popen(sim_command.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # Monitor stdout and stderr in separate threads.
    stdout_thread = threading.Thread(target=read_stream, args=(process.stdout, error_pub, error_keywords))
    stderr_thread = threading.Thread(target=read_stream, args=(process.stderr, error_pub, error_keywords))

    stdout_thread.start()
    stderr_thread.start()

    # Wait for the simulation process to finish
    process.wait()
    stdout_thread.join()
    stderr_thread.join()

    rospy.loginfo("Simulation process has ended.")

