;; Auto-generated. Do not edit!


(when (boundp 'racecar_cone_msgs::Cone)
  (if (not (find-package "RACECAR_CONE_MSGS"))
    (make-package "RACECAR_CONE_MSGS"))
  (shadow 'Cone (find-package "RACECAR_CONE_MSGS")))
(unless (find-package "RACECAR_CONE_MSGS::CONE")
  (make-package "RACECAR_CONE_MSGS::CONE"))

(in-package "ROS")
;;//! \htmlinclude Cone.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*UNKNOWN*" (find-package "RACECAR_CONE_MSGS::CONE"))
(shadow '*UNKNOWN* (find-package "RACECAR_CONE_MSGS::CONE"))
(defconstant racecar_cone_msgs::Cone::*UNKNOWN* 0)
(intern "*LEFT*" (find-package "RACECAR_CONE_MSGS::CONE"))
(shadow '*LEFT* (find-package "RACECAR_CONE_MSGS::CONE"))
(defconstant racecar_cone_msgs::Cone::*LEFT* 1)
(intern "*RIGHT*" (find-package "RACECAR_CONE_MSGS::CONE"))
(shadow '*RIGHT* (find-package "RACECAR_CONE_MSGS::CONE"))
(defconstant racecar_cone_msgs::Cone::*RIGHT* 2)
(defclass racecar_cone_msgs::Cone
  :super ros::object
  :slots (_header _p _cov_xy _side _confidence _source _id ))

(defmethod racecar_cone_msgs::Cone
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:p __p) (instance geometry_msgs::Point :init))
    ((:cov_xy __cov_xy) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:side __side) 0)
    ((:confidence __confidence) 0.0)
    ((:source __source) "")
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _p __p)
   (setq _cov_xy __cov_xy)
   (setq _side (round __side))
   (setq _confidence (float __confidence))
   (setq _source (string __source))
   (setq _id (round __id))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:p
   (&rest __p)
   (if (keywordp (car __p))
       (send* _p __p)
     (progn
       (if __p (setq _p (car __p)))
       _p)))
  (:cov_xy
   (&optional __cov_xy)
   (if __cov_xy (setq _cov_xy __cov_xy)) _cov_xy)
  (:side
   (&optional __side)
   (if __side (setq _side __side)) _side)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:source
   (&optional __source)
   (if __source (setq _source __source)) _source)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _p
    (send _p :serialization-length)
    ;; float32[4] _cov_xy
    (* 4    4)
    ;; uint8 _side
    1
    ;; float32 _confidence
    4
    ;; string _source
    4 (length _source)
    ;; int32 _id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _p
       (send _p :serialize s)
     ;; float32[4] _cov_xy
     (dotimes (i 4)
       (sys::poke (elt _cov_xy i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint8 _side
       (write-byte _side s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _source
       (write-long (length _source) s) (princ _source s)
     ;; int32 _id
       (write-long _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _p
     (send _p :deserialize buf ptr-) (incf ptr- (send _p :serialization-length))
   ;; float32[4] _cov_xy
   (dotimes (i (length _cov_xy))
     (setf (elt _cov_xy i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; uint8 _side
     (setq _side (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _source
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _source (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get racecar_cone_msgs::Cone :md5sum-) "ac79e60f67f210d41f97d0cd883ed6b5")
(setf (get racecar_cone_msgs::Cone :datatype-) "racecar_cone_msgs/Cone")
(setf (get racecar_cone_msgs::Cone :definition-)
      "# Detección de un cono (frame fijo: odom/map)
std_msgs/Header header
geometry_msgs/Point p           # posición (z=0 si 2D)
# Covarianza plana 2x2 (x,y) aplanada (row-major): [xx, xy, yx, yy]
float32[4] cov_xy

# Lateralidad
uint8 UNKNOWN=0
uint8 LEFT=1
uint8 RIGHT=2
uint8 side

float32 confidence              # [0..1]
string source                   # \"lidar\", \"sim\", etc.
int32 id                        # opcional (tracking)

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



(provide :racecar_cone_msgs/Cone "ac79e60f67f210d41f97d0cd883ed6b5")


