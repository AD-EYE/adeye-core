;; Auto-generated. Do not edit!


(when (boundp 'adeye_msgs::categorized_poses)
  (if (not (find-package "ADEYE_MSGS"))
    (make-package "ADEYE_MSGS"))
  (shadow 'categorized_poses (find-package "ADEYE_MSGS")))
(unless (find-package "ADEYE_MSGS::CATEGORIZED_POSES")
  (make-package "ADEYE_MSGS::CATEGORIZED_POSES"))

(in-package "ROS")
;;//! \htmlinclude categorized_poses.msg.html


(defclass adeye_msgs::categorized_poses
  :super ros::object
  :slots (_poses ))

(defmethod adeye_msgs::categorized_poses
  (:init
   (&key
    ((:poses __poses) (let (r) (dotimes (i 0) (push (instance adeye_msgs::categorized_pose :init) r)) r))
    )
   (send-super :init)
   (setq _poses __poses)
   self)
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
  (:serialization-length
   ()
   (+
    ;; adeye_msgs/categorized_pose[] _poses
    (apply #'+ (send-all _poses :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; adeye_msgs/categorized_pose[] _poses
     (write-long (length _poses) s)
     (dolist (elem _poses)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; adeye_msgs/categorized_pose[] _poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poses (let (r) (dotimes (i n) (push (instance adeye_msgs::categorized_pose :init) r)) r))
     (dolist (elem- _poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get adeye_msgs::categorized_poses :md5sum-) "48191d44253f84ab78879ef888b206c3")
(setf (get adeye_msgs::categorized_poses :datatype-) "adeye_msgs/categorized_poses")
(setf (get adeye_msgs::categorized_poses :definition-)
      "categorized_pose[] poses
================================================================================
MSG: adeye_msgs/categorized_pose
Header header
int64 category
geometry_msgs/Pose pose
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :adeye_msgs/categorized_poses "48191d44253f84ab78879ef888b206c3")


