;; Auto-generated. Do not edit!


(when (boundp 'racecar_cone_msgs::ConePair)
  (if (not (find-package "RACECAR_CONE_MSGS"))
    (make-package "RACECAR_CONE_MSGS"))
  (shadow 'ConePair (find-package "RACECAR_CONE_MSGS")))
(unless (find-package "RACECAR_CONE_MSGS::CONEPAIR")
  (make-package "RACECAR_CONE_MSGS::CONEPAIR"))

(in-package "ROS")
;;//! \htmlinclude ConePair.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass racecar_cone_msgs::ConePair
  :super ros::object
  :slots (_header _left _right _midpoint _width ))

(defmethod racecar_cone_msgs::ConePair
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left __left) (instance racecar_cone_msgs::Cone :init))
    ((:right __right) (instance racecar_cone_msgs::Cone :init))
    ((:midpoint __midpoint) (instance geometry_msgs::Point :init))
    ((:width __width) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _left __left)
   (setq _right __right)
   (setq _midpoint __midpoint)
   (setq _width (float __width))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left
   (&rest __left)
   (if (keywordp (car __left))
       (send* _left __left)
     (progn
       (if __left (setq _left (car __left)))
       _left)))
  (:right
   (&rest __right)
   (if (keywordp (car __right))
       (send* _right __right)
     (progn
       (if __right (setq _right (car __right)))
       _right)))
  (:midpoint
   (&rest __midpoint)
   (if (keywordp (car __midpoint))
       (send* _midpoint __midpoint)
     (progn
       (if __midpoint (setq _midpoint (car __midpoint)))
       _midpoint)))
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; racecar_cone_msgs/Cone _left
    (send _left :serialization-length)
    ;; racecar_cone_msgs/Cone _right
    (send _right :serialization-length)
    ;; geometry_msgs/Point _midpoint
    (send _midpoint :serialization-length)
    ;; float32 _width
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; racecar_cone_msgs/Cone _left
       (send _left :serialize s)
     ;; racecar_cone_msgs/Cone _right
       (send _right :serialize s)
     ;; geometry_msgs/Point _midpoint
       (send _midpoint :serialize s)
     ;; float32 _width
       (sys::poke _width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; racecar_cone_msgs/Cone _left
     (send _left :deserialize buf ptr-) (incf ptr- (send _left :serialization-length))
   ;; racecar_cone_msgs/Cone _right
     (send _right :deserialize buf ptr-) (incf ptr- (send _right :serialization-length))
   ;; geometry_msgs/Point _midpoint
     (send _midpoint :deserialize buf ptr-) (incf ptr- (send _midpoint :serialization-length))
   ;; float32 _width
     (setq _width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get racecar_cone_msgs::ConePair :md5sum-) "e86328cf0c5c9f6e1d43796e78dbb005")
(setf (get racecar_cone_msgs::ConePair :datatype-) "racecar_cone_msgs/ConePair")
(setf (get racecar_cone_msgs::ConePair :definition-)
      "std_msgs/Header header
racecar_cone_msgs/Cone left
racecar_cone_msgs/Cone right
geometry_msgs/Point midpoint
float32 width                   # ||right - left||

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
MSG: racecar_cone_msgs/Cone
# Detección de un cono (frame fijo: odom/map)
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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :racecar_cone_msgs/ConePair "e86328cf0c5c9f6e1d43796e78dbb005")


