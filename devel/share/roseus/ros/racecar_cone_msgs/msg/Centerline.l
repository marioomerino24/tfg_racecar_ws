;; Auto-generated. Do not edit!


(when (boundp 'racecar_cone_msgs::Centerline)
  (if (not (find-package "RACECAR_CONE_MSGS"))
    (make-package "RACECAR_CONE_MSGS"))
  (shadow 'Centerline (find-package "RACECAR_CONE_MSGS")))
(unless (find-package "RACECAR_CONE_MSGS::CENTERLINE")
  (make-package "RACECAR_CONE_MSGS::CENTERLINE"))

(in-package "ROS")
;;//! \htmlinclude Centerline.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass racecar_cone_msgs::Centerline
  :super ros::object
  :slots (_header _samples _s _kappa _is_arclength_uniform _ds _ctrl_pts ))

(defmethod racecar_cone_msgs::Centerline
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:samples __samples) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:s __s) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:kappa __kappa) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:is_arclength_uniform __is_arclength_uniform) nil)
    ((:ds __ds) 0.0)
    ((:ctrl_pts __ctrl_pts) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _samples __samples)
   (setq _s __s)
   (setq _kappa __kappa)
   (setq _is_arclength_uniform __is_arclength_uniform)
   (setq _ds (float __ds))
   (setq _ctrl_pts __ctrl_pts)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:samples
   (&rest __samples)
   (if (keywordp (car __samples))
       (send* _samples __samples)
     (progn
       (if __samples (setq _samples (car __samples)))
       _samples)))
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:kappa
   (&optional __kappa)
   (if __kappa (setq _kappa __kappa)) _kappa)
  (:is_arclength_uniform
   (&optional __is_arclength_uniform)
   (if __is_arclength_uniform (setq _is_arclength_uniform __is_arclength_uniform)) _is_arclength_uniform)
  (:ds
   (&optional __ds)
   (if __ds (setq _ds __ds)) _ds)
  (:ctrl_pts
   (&rest __ctrl_pts)
   (if (keywordp (car __ctrl_pts))
       (send* _ctrl_pts __ctrl_pts)
     (progn
       (if __ctrl_pts (setq _ctrl_pts (car __ctrl_pts)))
       _ctrl_pts)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point[] _samples
    (apply #'+ (send-all _samples :serialization-length)) 4
    ;; float32[] _s
    (* 4    (length _s)) 4
    ;; float32[] _kappa
    (* 4    (length _kappa)) 4
    ;; bool _is_arclength_uniform
    1
    ;; float32 _ds
    4
    ;; geometry_msgs/Point[] _ctrl_pts
    (apply #'+ (send-all _ctrl_pts :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point[] _samples
     (write-long (length _samples) s)
     (dolist (elem _samples)
       (send elem :serialize s)
       )
     ;; float32[] _s
     (write-long (length _s) s)
     (dotimes (i (length _s))
       (sys::poke (elt _s i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _kappa
     (write-long (length _kappa) s)
     (dotimes (i (length _kappa))
       (sys::poke (elt _kappa i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _is_arclength_uniform
       (if _is_arclength_uniform (write-byte -1 s) (write-byte 0 s))
     ;; float32 _ds
       (sys::poke _ds (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Point[] _ctrl_pts
     (write-long (length _ctrl_pts) s)
     (dolist (elem _ctrl_pts)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point[] _samples
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _samples (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _samples)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32[] _s
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _s (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _s i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _kappa
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _kappa (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _kappa i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _is_arclength_uniform
     (setq _is_arclength_uniform (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _ds
     (setq _ds (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Point[] _ctrl_pts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ctrl_pts (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _ctrl_pts)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get racecar_cone_msgs::Centerline :md5sum-) "3934b303e0826c747777abfdd52e228c")
(setf (get racecar_cone_msgs::Centerline :datatype-) "racecar_cone_msgs/Centerline")
(setf (get racecar_cone_msgs::Centerline :definition-)
      "std_msgs/Header header                 # frame_id típico: \"odom\"

# Muestras de la trayectoria c(s) remuestreadas
geometry_msgs/Point[] samples          # tamaño N

# Longitud de arco acumulada para cada sample (m)
float32[] s                            # tamaño N, s[0]=0

# Curvatura por muestra (1/m). Signo según convención (yaw izquierda +)
float32[] kappa                        # tamaño N

# Metadatos de muestreo
bool is_arclength_uniform              # true si s es uniforme
float32 ds                             # paso nominal en s (m), 0 si no aplica

# (Opcional) puntos de control del ajuste (vacío si no aplica)
geometry_msgs/Point[] ctrl_pts

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
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :racecar_cone_msgs/Centerline "3934b303e0826c747777abfdd52e228c")


