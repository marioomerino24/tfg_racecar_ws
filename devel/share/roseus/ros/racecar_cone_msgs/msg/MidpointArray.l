;; Auto-generated. Do not edit!


(when (boundp 'racecar_cone_msgs::MidpointArray)
  (if (not (find-package "RACECAR_CONE_MSGS"))
    (make-package "RACECAR_CONE_MSGS"))
  (shadow 'MidpointArray (find-package "RACECAR_CONE_MSGS")))
(unless (find-package "RACECAR_CONE_MSGS::MIDPOINTARRAY")
  (make-package "RACECAR_CONE_MSGS::MIDPOINTARRAY"))

(in-package "ROS")
;;//! \htmlinclude MidpointArray.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass racecar_cone_msgs::MidpointArray
  :super ros::object
  :slots (_header _points _width _cov_xy_flat _id _confidence ))

(defmethod racecar_cone_msgs::MidpointArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:points __points) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:width __width) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cov_xy_flat __cov_xy_flat) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:id __id) (make-array 0 :initial-element 0 :element-type :integer))
    ((:confidence __confidence) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _points __points)
   (setq _width __width)
   (setq _cov_xy_flat __cov_xy_flat)
   (setq _id __id)
   (setq _confidence __confidence)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:cov_xy_flat
   (&optional __cov_xy_flat)
   (if __cov_xy_flat (setq _cov_xy_flat __cov_xy_flat)) _cov_xy_flat)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ;; float32[] _width
    (* 4    (length _width)) 4
    ;; float32[] _cov_xy_flat
    (* 4    (length _cov_xy_flat)) 4
    ;; int32[] _id
    (* 4    (length _id)) 4
    ;; float32[] _confidence
    (* 4    (length _confidence)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;; float32[] _width
     (write-long (length _width) s)
     (dotimes (i (length _width))
       (sys::poke (elt _width i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _cov_xy_flat
     (write-long (length _cov_xy_flat) s)
     (dotimes (i (length _cov_xy_flat))
       (sys::poke (elt _cov_xy_flat i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int32[] _id
     (write-long (length _id) s)
     (dotimes (i (length _id))
       (write-long (elt _id i) s)
       )
     ;; float32[] _confidence
     (write-long (length _confidence) s)
     (dotimes (i (length _confidence))
       (sys::poke (elt _confidence i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32[] _width
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _width (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _width i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _cov_xy_flat
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cov_xy_flat (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cov_xy_flat i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int32[] _id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _id (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _id i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32[] _confidence
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _confidence (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _confidence i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get racecar_cone_msgs::MidpointArray :md5sum-) "b6b5155c16724caf7952716fccf9a8ba")
(setf (get racecar_cone_msgs::MidpointArray :datatype-) "racecar_cone_msgs/MidpointArray")
(setf (get racecar_cone_msgs::MidpointArray :definition-)
      "std_msgs/Header header

# Puntos medios ordenados por avance
geometry_msgs/Point[] points

# Ancho local asociado a cada punto (m)
float32[] width

# Covarianza 2x2 de cada midpoint, aplanada (row-major) en bloques de 4:
# [xx, xy, yx, yy,  xx, xy, yx, yy, ...]  => longitud = 4 * N
float32[] cov_xy_flat

# IDs y confianza opcionales
int32[] id
float32[] confidence

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



(provide :racecar_cone_msgs/MidpointArray "b6b5155c16724caf7952716fccf9a8ba")


