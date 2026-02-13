; Auto-generated. Do not edit!


(cl:in-package racecar_cone_msgs-msg)


;//! \htmlinclude ConePair.msg.html

(cl:defclass <ConePair> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left
    :reader left
    :initarg :left
    :type racecar_cone_msgs-msg:Cone
    :initform (cl:make-instance 'racecar_cone_msgs-msg:Cone))
   (right
    :reader right
    :initarg :right
    :type racecar_cone_msgs-msg:Cone
    :initform (cl:make-instance 'racecar_cone_msgs-msg:Cone))
   (midpoint
    :reader midpoint
    :initarg :midpoint
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (width
    :reader width
    :initarg :width
    :type cl:float
    :initform 0.0))
)

(cl:defclass ConePair (<ConePair>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConePair>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConePair)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name racecar_cone_msgs-msg:<ConePair> is deprecated: use racecar_cone_msgs-msg:ConePair instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ConePair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:header-val is deprecated.  Use racecar_cone_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <ConePair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:left-val is deprecated.  Use racecar_cone_msgs-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <ConePair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:right-val is deprecated.  Use racecar_cone_msgs-msg:right instead.")
  (right m))

(cl:ensure-generic-function 'midpoint-val :lambda-list '(m))
(cl:defmethod midpoint-val ((m <ConePair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:midpoint-val is deprecated.  Use racecar_cone_msgs-msg:midpoint instead.")
  (midpoint m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <ConePair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:width-val is deprecated.  Use racecar_cone_msgs-msg:width instead.")
  (width m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConePair>) ostream)
  "Serializes a message object of type '<ConePair>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'left) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'right) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'midpoint) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConePair>) istream)
  "Deserializes a message object of type '<ConePair>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'left) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'right) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'midpoint) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConePair>)))
  "Returns string type for a message object of type '<ConePair>"
  "racecar_cone_msgs/ConePair")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConePair)))
  "Returns string type for a message object of type 'ConePair"
  "racecar_cone_msgs/ConePair")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConePair>)))
  "Returns md5sum for a message object of type '<ConePair>"
  "e86328cf0c5c9f6e1d43796e78dbb005")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConePair)))
  "Returns md5sum for a message object of type 'ConePair"
  "e86328cf0c5c9f6e1d43796e78dbb005")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConePair>)))
  "Returns full string definition for message of type '<ConePair>"
  (cl:format cl:nil "std_msgs/Header header~%racecar_cone_msgs/Cone left~%racecar_cone_msgs/Cone right~%geometry_msgs/Point midpoint~%float32 width                   # ||right - left||~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: racecar_cone_msgs/Cone~%# Detección de un cono (frame fijo: odom/map)~%std_msgs/Header header~%geometry_msgs/Point p           # posición (z=0 si 2D)~%# Covarianza plana 2x2 (x,y) aplanada (row-major): [xx, xy, yx, yy]~%float32[4] cov_xy~%~%# Lateralidad~%uint8 UNKNOWN=0~%uint8 LEFT=1~%uint8 RIGHT=2~%uint8 side~%~%float32 confidence              # [0..1]~%string source                   # \"lidar\", \"sim\", etc.~%int32 id                        # opcional (tracking)~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConePair)))
  "Returns full string definition for message of type 'ConePair"
  (cl:format cl:nil "std_msgs/Header header~%racecar_cone_msgs/Cone left~%racecar_cone_msgs/Cone right~%geometry_msgs/Point midpoint~%float32 width                   # ||right - left||~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: racecar_cone_msgs/Cone~%# Detección de un cono (frame fijo: odom/map)~%std_msgs/Header header~%geometry_msgs/Point p           # posición (z=0 si 2D)~%# Covarianza plana 2x2 (x,y) aplanada (row-major): [xx, xy, yx, yy]~%float32[4] cov_xy~%~%# Lateralidad~%uint8 UNKNOWN=0~%uint8 LEFT=1~%uint8 RIGHT=2~%uint8 side~%~%float32 confidence              # [0..1]~%string source                   # \"lidar\", \"sim\", etc.~%int32 id                        # opcional (tracking)~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConePair>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'left))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'right))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'midpoint))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConePair>))
  "Converts a ROS message object to a list"
  (cl:list 'ConePair
    (cl:cons ':header (header msg))
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
    (cl:cons ':midpoint (midpoint msg))
    (cl:cons ':width (width msg))
))
