; Auto-generated. Do not edit!


(cl:in-package racecar_cone_msgs-msg)


;//! \htmlinclude Centerline.msg.html

(cl:defclass <Centerline> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (samples
    :reader samples
    :initarg :samples
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (s
    :reader s
    :initarg :s
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (kappa
    :reader kappa
    :initarg :kappa
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (is_arclength_uniform
    :reader is_arclength_uniform
    :initarg :is_arclength_uniform
    :type cl:boolean
    :initform cl:nil)
   (ds
    :reader ds
    :initarg :ds
    :type cl:float
    :initform 0.0)
   (ctrl_pts
    :reader ctrl_pts
    :initarg :ctrl_pts
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass Centerline (<Centerline>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Centerline>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Centerline)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name racecar_cone_msgs-msg:<Centerline> is deprecated: use racecar_cone_msgs-msg:Centerline instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Centerline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:header-val is deprecated.  Use racecar_cone_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'samples-val :lambda-list '(m))
(cl:defmethod samples-val ((m <Centerline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:samples-val is deprecated.  Use racecar_cone_msgs-msg:samples instead.")
  (samples m))

(cl:ensure-generic-function 's-val :lambda-list '(m))
(cl:defmethod s-val ((m <Centerline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:s-val is deprecated.  Use racecar_cone_msgs-msg:s instead.")
  (s m))

(cl:ensure-generic-function 'kappa-val :lambda-list '(m))
(cl:defmethod kappa-val ((m <Centerline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:kappa-val is deprecated.  Use racecar_cone_msgs-msg:kappa instead.")
  (kappa m))

(cl:ensure-generic-function 'is_arclength_uniform-val :lambda-list '(m))
(cl:defmethod is_arclength_uniform-val ((m <Centerline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:is_arclength_uniform-val is deprecated.  Use racecar_cone_msgs-msg:is_arclength_uniform instead.")
  (is_arclength_uniform m))

(cl:ensure-generic-function 'ds-val :lambda-list '(m))
(cl:defmethod ds-val ((m <Centerline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:ds-val is deprecated.  Use racecar_cone_msgs-msg:ds instead.")
  (ds m))

(cl:ensure-generic-function 'ctrl_pts-val :lambda-list '(m))
(cl:defmethod ctrl_pts-val ((m <Centerline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader racecar_cone_msgs-msg:ctrl_pts-val is deprecated.  Use racecar_cone_msgs-msg:ctrl_pts instead.")
  (ctrl_pts m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Centerline>) ostream)
  "Serializes a message object of type '<Centerline>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'samples))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'samples))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 's))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 's))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'kappa))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'kappa))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_arclength_uniform) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ctrl_pts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'ctrl_pts))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Centerline>) istream)
  "Deserializes a message object of type '<Centerline>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'samples) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'samples)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 's) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 's)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'kappa) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'kappa)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'is_arclength_uniform) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ds) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ctrl_pts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ctrl_pts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Centerline>)))
  "Returns string type for a message object of type '<Centerline>"
  "racecar_cone_msgs/Centerline")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Centerline)))
  "Returns string type for a message object of type 'Centerline"
  "racecar_cone_msgs/Centerline")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Centerline>)))
  "Returns md5sum for a message object of type '<Centerline>"
  "3934b303e0826c747777abfdd52e228c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Centerline)))
  "Returns md5sum for a message object of type 'Centerline"
  "3934b303e0826c747777abfdd52e228c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Centerline>)))
  "Returns full string definition for message of type '<Centerline>"
  (cl:format cl:nil "std_msgs/Header header                 # frame_id típico: \"odom\"~%~%# Muestras de la trayectoria c(s) remuestreadas~%geometry_msgs/Point[] samples          # tamaño N~%~%# Longitud de arco acumulada para cada sample (m)~%float32[] s                            # tamaño N, s[0]=0~%~%# Curvatura por muestra (1/m). Signo según convención (yaw izquierda +)~%float32[] kappa                        # tamaño N~%~%# Metadatos de muestreo~%bool is_arclength_uniform              # true si s es uniforme~%float32 ds                             # paso nominal en s (m), 0 si no aplica~%~%# (Opcional) puntos de control del ajuste (vacío si no aplica)~%geometry_msgs/Point[] ctrl_pts~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Centerline)))
  "Returns full string definition for message of type 'Centerline"
  (cl:format cl:nil "std_msgs/Header header                 # frame_id típico: \"odom\"~%~%# Muestras de la trayectoria c(s) remuestreadas~%geometry_msgs/Point[] samples          # tamaño N~%~%# Longitud de arco acumulada para cada sample (m)~%float32[] s                            # tamaño N, s[0]=0~%~%# Curvatura por muestra (1/m). Signo según convención (yaw izquierda +)~%float32[] kappa                        # tamaño N~%~%# Metadatos de muestreo~%bool is_arclength_uniform              # true si s es uniforme~%float32 ds                             # paso nominal en s (m), 0 si no aplica~%~%# (Opcional) puntos de control del ajuste (vacío si no aplica)~%geometry_msgs/Point[] ctrl_pts~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Centerline>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'samples) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 's) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'kappa) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ctrl_pts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Centerline>))
  "Converts a ROS message object to a list"
  (cl:list 'Centerline
    (cl:cons ':header (header msg))
    (cl:cons ':samples (samples msg))
    (cl:cons ':s (s msg))
    (cl:cons ':kappa (kappa msg))
    (cl:cons ':is_arclength_uniform (is_arclength_uniform msg))
    (cl:cons ':ds (ds msg))
    (cl:cons ':ctrl_pts (ctrl_pts msg))
))
