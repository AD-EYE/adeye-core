; Auto-generated. Do not edit!


(cl:in-package adeye_msgs-msg)


;//! \htmlinclude categorized_pose.msg.html

(cl:defclass <categorized_pose> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (category
    :reader category
    :initarg :category
    :type cl:integer
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass categorized_pose (<categorized_pose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <categorized_pose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'categorized_pose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name adeye_msgs-msg:<categorized_pose> is deprecated: use adeye_msgs-msg:categorized_pose instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <categorized_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adeye_msgs-msg:header-val is deprecated.  Use adeye_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'category-val :lambda-list '(m))
(cl:defmethod category-val ((m <categorized_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adeye_msgs-msg:category-val is deprecated.  Use adeye_msgs-msg:category instead.")
  (category m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <categorized_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adeye_msgs-msg:pose-val is deprecated.  Use adeye_msgs-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <categorized_pose>) ostream)
  "Serializes a message object of type '<categorized_pose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'category)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <categorized_pose>) istream)
  "Deserializes a message object of type '<categorized_pose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'category) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<categorized_pose>)))
  "Returns string type for a message object of type '<categorized_pose>"
  "adeye_msgs/categorized_pose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'categorized_pose)))
  "Returns string type for a message object of type 'categorized_pose"
  "adeye_msgs/categorized_pose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<categorized_pose>)))
  "Returns md5sum for a message object of type '<categorized_pose>"
  "7ea9955f5189fbec2a19e42fe9a37790")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'categorized_pose)))
  "Returns md5sum for a message object of type 'categorized_pose"
  "7ea9955f5189fbec2a19e42fe9a37790")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<categorized_pose>)))
  "Returns full string definition for message of type '<categorized_pose>"
  (cl:format cl:nil "Header header~%int64 category~%geometry_msgs/Pose pose~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'categorized_pose)))
  "Returns full string definition for message of type 'categorized_pose"
  (cl:format cl:nil "Header header~%int64 category~%geometry_msgs/Pose pose~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <categorized_pose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <categorized_pose>))
  "Converts a ROS message object to a list"
  (cl:list 'categorized_pose
    (cl:cons ':header (header msg))
    (cl:cons ':category (category msg))
    (cl:cons ':pose (pose msg))
))
