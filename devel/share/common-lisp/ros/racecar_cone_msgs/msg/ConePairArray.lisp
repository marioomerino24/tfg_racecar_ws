; Auto-generated. Do not edit!


(cl:in-package racecar_cone_msgs-msg)


;//! \htmlinclude ConePairArray.msg.html

(cl:defclass <ConePairArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pairs
    :reader pairs
    :initarg :pairs
    :type (cl:vector racecar_cone_msgs-msg:ConePair)
   :initform (cl:make-array 0 :element-type 'racecar_cone_msgs-msg:ConePair :initial-element (cl:make-instance 'racecar_cone_msgs-msg:ConePair))))
)

(cl:defclass ConePairArray (<ConePairArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConePairArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConePairArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name racecar_cone_msgs-msg:<ConePairArray> is deprecated: use racecar_cone_msgs-msg:ConePairArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ConePairArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:header-val is deprecated.  Use racecar_cone_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pairs-val :lambda-list '(m))
(cl:defmethod pairs-val ((m <ConePairArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:pairs-val is deprecated.  Use racecar_cone_msgs-msg:pairs instead.")
  (pairs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConePairArray>) ostream)
  "Serializes a message object of type '<ConePairArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pairs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'pairs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConePairArray>) istream)
  "Deserializes a message object of type '<ConePairArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pairs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pairs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'racecar_cone_msgs-msg:ConePair))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConePairArray>)))
  "Returns string type for a message object of type '<ConePairArray>"
  "racecar_cone_msgs/ConePairArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConePairArray)))
  "Returns string type for a message object of type 'ConePairArray"
  "racecar_cone_msgs/ConePairArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConePairArray>)))
  "Returns md5sum for a message object of type '<ConePairArray>"
  "fbdccabe23e08b5ded654d3910450cdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConePairArray)))
  "Returns md5sum for a message object of type 'ConePairArray"
  "fbdccabe23e08b5ded654d3910450cdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConePairArray>)))
  "Returns full string definition for message of type '<ConePairArray>"
  (cl:format cl:nil "std_msgs/Header header~%racecar_cone_msgs/ConePair[] pairs~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: racecar_cone_msgs/ConePair~%std_msgs/Header header~%racecar_cone_msgs/Cone left~%racecar_cone_msgs/Cone right~%geometry_msgs/Point midpoint~%float32 width                   # ||right - left||~%~%================================================================================~%MSG: racecar_cone_msgs/Cone~%# Detección de un cono (frame fijo: odom/map)~%std_msgs/Header header~%geometry_msgs/Point p           # posición (z=0 si 2D)~%# Covarianza plana 2x2 (x,y) aplanada (row-major): [xx, xy, yx, yy]~%float32[4] cov_xy~%~%# Lateralidad~%uint8 UNKNOWN=0~%uint8 LEFT=1~%uint8 RIGHT=2~%uint8 side~%~%float32 confidence              # [0..1]~%string source                   # \"lidar\", \"sim\", etc.~%int32 id                        # opcional (tracking)~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConePairArray)))
  "Returns full string definition for message of type 'ConePairArray"
  (cl:format cl:nil "std_msgs/Header header~%racecar_cone_msgs/ConePair[] pairs~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: racecar_cone_msgs/ConePair~%std_msgs/Header header~%racecar_cone_msgs/Cone left~%racecar_cone_msgs/Cone right~%geometry_msgs/Point midpoint~%float32 width                   # ||right - left||~%~%================================================================================~%MSG: racecar_cone_msgs/Cone~%# Detección de un cono (frame fijo: odom/map)~%std_msgs/Header header~%geometry_msgs/Point p           # posición (z=0 si 2D)~%# Covarianza plana 2x2 (x,y) aplanada (row-major): [xx, xy, yx, yy]~%float32[4] cov_xy~%~%# Lateralidad~%uint8 UNKNOWN=0~%uint8 LEFT=1~%uint8 RIGHT=2~%uint8 side~%~%float32 confidence              # [0..1]~%string source                   # \"lidar\", \"sim\", etc.~%int32 id                        # opcional (tracking)~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConePairArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pairs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConePairArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ConePairArray
    (cl:cons ':header (header msg))
    (cl:cons ':pairs (pairs msg))
))
