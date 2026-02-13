; Auto-generated. Do not edit!


(cl:in-package racecar_cone_msgs-msg)


;//! \htmlinclude TrackMetrics.msg.html

(cl:defclass <TrackMetrics> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (coverage
    :reader coverage
    :initarg :coverage
    :type cl:float
    :initform 0.0)
   (std_width
    :reader std_width
    :initarg :std_width
    :type cl:float
    :initform 0.0)
   (max_kappa
    :reader max_kappa
    :initarg :max_kappa
    :type cl:float
    :initform 0.0)
   (integral_kappa_sq
    :reader integral_kappa_sq
    :initarg :integral_kappa_sq
    :type cl:float
    :initform 0.0)
   (latency_ms
    :reader latency_ms
    :initarg :latency_ms
    :type cl:float
    :initform 0.0)
   (cones_raw_count
    :reader cones_raw_count
    :initarg :cones_raw_count
    :type cl:integer
    :initform 0)
   (pairs_count
    :reader pairs_count
    :initarg :pairs_count
    :type cl:integer
    :initform 0)
   (midpoints_count
    :reader midpoints_count
    :initarg :midpoints_count
    :type cl:integer
    :initform 0)
   (notes
    :reader notes
    :initarg :notes
    :type cl:string
    :initform ""))
)

(cl:defclass TrackMetrics (<TrackMetrics>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackMetrics>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackMetrics)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name racecar_cone_msgs-msg:<TrackMetrics> is deprecated: use racecar_cone_msgs-msg:TrackMetrics instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrackMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:header-val is deprecated.  Use racecar_cone_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'coverage-val :lambda-list '(m))
(cl:defmethod coverage-val ((m <TrackMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:coverage-val is deprecated.  Use racecar_cone_msgs-msg:coverage instead.")
  (coverage m))

(cl:ensure-generic-function 'std_width-val :lambda-list '(m))
(cl:defmethod std_width-val ((m <TrackMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:std_width-val is deprecated.  Use racecar_cone_msgs-msg:std_width instead.")
  (std_width m))

(cl:ensure-generic-function 'max_kappa-val :lambda-list '(m))
(cl:defmethod max_kappa-val ((m <TrackMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:max_kappa-val is deprecated.  Use racecar_cone_msgs-msg:max_kappa instead.")
  (max_kappa m))

(cl:ensure-generic-function 'integral_kappa_sq-val :lambda-list '(m))
(cl:defmethod integral_kappa_sq-val ((m <TrackMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:integral_kappa_sq-val is deprecated.  Use racecar_cone_msgs-msg:integral_kappa_sq instead.")
  (integral_kappa_sq m))

(cl:ensure-generic-function 'latency_ms-val :lambda-list '(m))
(cl:defmethod latency_ms-val ((m <TrackMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:latency_ms-val is deprecated.  Use racecar_cone_msgs-msg:latency_ms instead.")
  (latency_ms m))

(cl:ensure-generic-function 'cones_raw_count-val :lambda-list '(m))
(cl:defmethod cones_raw_count-val ((m <TrackMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:cones_raw_count-val is deprecated.  Use racecar_cone_msgs-msg:cones_raw_count instead.")
  (cones_raw_count m))

(cl:ensure-generic-function 'pairs_count-val :lambda-list '(m))
(cl:defmethod pairs_count-val ((m <TrackMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:pairs_count-val is deprecated.  Use racecar_cone_msgs-msg:pairs_count instead.")
  (pairs_count m))

(cl:ensure-generic-function 'midpoints_count-val :lambda-list '(m))
(cl:defmethod midpoints_count-val ((m <TrackMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:midpoints_count-val is deprecated.  Use racecar_cone_msgs-msg:midpoints_count instead.")
  (midpoints_count m))

(cl:ensure-generic-function 'notes-val :lambda-list '(m))
(cl:defmethod notes-val ((m <TrackMetrics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:notes-val is deprecated.  Use racecar_cone_msgs-msg:notes instead.")
  (notes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackMetrics>) ostream)
  "Serializes a message object of type '<TrackMetrics>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'coverage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'std_width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_kappa))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'integral_kappa_sq))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latency_ms))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cones_raw_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cones_raw_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cones_raw_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cones_raw_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pairs_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pairs_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pairs_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pairs_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'midpoints_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'midpoints_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'midpoints_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'midpoints_count)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'notes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'notes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackMetrics>) istream)
  "Deserializes a message object of type '<TrackMetrics>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'coverage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'std_width) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_kappa) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'integral_kappa_sq) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latency_ms) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cones_raw_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cones_raw_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cones_raw_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cones_raw_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pairs_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pairs_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pairs_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pairs_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'midpoints_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'midpoints_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'midpoints_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'midpoints_count)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'notes) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'notes) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackMetrics>)))
  "Returns string type for a message object of type '<TrackMetrics>"
  "racecar_cone_msgs/TrackMetrics")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackMetrics)))
  "Returns string type for a message object of type 'TrackMetrics"
  "racecar_cone_msgs/TrackMetrics")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackMetrics>)))
  "Returns md5sum for a message object of type '<TrackMetrics>"
  "9f81acf08fbcb042f5be8a05aaee3bf6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackMetrics)))
  "Returns md5sum for a message object of type 'TrackMetrics"
  "9f81acf08fbcb042f5be8a05aaee3bf6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackMetrics>)))
  "Returns full string definition for message of type '<TrackMetrics>"
  (cl:format cl:nil "std_msgs/Header header~%~%# Cobertura de emparejado (0..1)~%float32 coverage~%~%# Estadística de anchos~%float32 std_width                      # desviación estándar de width (m)~%~%# Curvatura y suavidad~%float32 max_kappa                      # max |kappa| (1/m)~%float32 integral_kappa_sq              # ∫ kappa^2 ds (1/m)~%~%# Rendimiento~%float32 latency_ms                     # latencia media del ciclo (ms)~%~%# Contadores~%uint32 cones_raw_count~%uint32 pairs_count~%uint32 midpoints_count~%~%# Notas/diagnóstico~%string notes~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackMetrics)))
  "Returns full string definition for message of type 'TrackMetrics"
  (cl:format cl:nil "std_msgs/Header header~%~%# Cobertura de emparejado (0..1)~%float32 coverage~%~%# Estadística de anchos~%float32 std_width                      # desviación estándar de width (m)~%~%# Curvatura y suavidad~%float32 max_kappa                      # max |kappa| (1/m)~%float32 integral_kappa_sq              # ∫ kappa^2 ds (1/m)~%~%# Rendimiento~%float32 latency_ms                     # latencia media del ciclo (ms)~%~%# Contadores~%uint32 cones_raw_count~%uint32 pairs_count~%uint32 midpoints_count~%~%# Notas/diagnóstico~%string notes~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackMetrics>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'notes))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackMetrics>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackMetrics
    (cl:cons ':header (header msg))
    (cl:cons ':coverage (coverage msg))
    (cl:cons ':std_width (std_width msg))
    (cl:cons ':max_kappa (max_kappa msg))
    (cl:cons ':integral_kappa_sq (integral_kappa_sq msg))
    (cl:cons ':latency_ms (latency_ms msg))
    (cl:cons ':cones_raw_count (cones_raw_count msg))
    (cl:cons ':pairs_count (pairs_count msg))
    (cl:cons ':midpoints_count (midpoints_count msg))
    (cl:cons ':notes (notes msg))
))
