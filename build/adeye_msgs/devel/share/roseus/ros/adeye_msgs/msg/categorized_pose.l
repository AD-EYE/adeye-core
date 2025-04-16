;; Auto-generated. Do not edit!


(when (boundp 'adeye_msgs::categorized_pose)
  (if (not (find-package "ADEYE_MSGS"))
    (make-package "ADEYE_MSGS"))
  (shadow 'categorized_pose (find-package "ADEYE_MSGS")))
(unless (find-package "ADEYE_MSGS::CATEGORIZED_POSE")
  (make-package "ADEYE_MSGS::CATEGORIZED_POSE"))

(in-package "ROS")
;;//! \htmlinclude categorized_pose.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass adeye_msgs::categorized_pose
  :super ros::object
  :slots (_header _category _pose ))

(defmethod adeye_msgs::categorized_pose
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:category __category) 0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _category (round __category))
   (setq _pose __pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:category
   (&optional __category)
   (if __category (setq _category __category)) _category)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int64 _category
    8
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int64 _category
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _category (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _category) (= (length (_category . bv)) 2)) ;; bignum
              (write-long (ash (elt (_category . bv) 0) 0) s)
              (write-long (ash (elt (_category . bv) 1) -1) s))
             ((and (class _category) (= (length (_category . bv)) 1)) ;; big1
              (write-long (elt (_category . bv) 0) s)
              (write-long (if (>= _category 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _category s)(write-long (if (>= _category 0) 0 #xffffffff) s)))
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int64 _category
#+(or :alpha :irix6 :x86_64)
      (setf _category (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _category (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get adeye_msgs::categorized_pose :md5sum-) "7ea9955f5189fbec2a19e42fe9a37790")
(setf (get adeye_msgs::categorized_pose :datatype-) "adeye_msgs/categorized_pose")
(setf (get adeye_msgs::categorized_pose :definition-)
      "Header header
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



(provide :adeye_msgs/categorized_pose "7ea9955f5189fbec2a19e42fe9a37790")


