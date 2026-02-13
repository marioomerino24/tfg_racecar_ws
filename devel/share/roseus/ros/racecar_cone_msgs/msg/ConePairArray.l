;; Auto-generated. Do not edit!


(when (boundp 'racecar_cone_msgs::ConePairArray)
  (if (not (find-package "RACECAR_CONE_MSGS"))
    (make-package "RACECAR_CONE_MSGS"))
  (shadow 'ConePairArray (find-package "RACECAR_CONE_MSGS")))
(unless (find-package "RACECAR_CONE_MSGS::CONEPAIRARRAY")
  (make-package "RACECAR_CONE_MSGS::CONEPAIRARRAY"))

(in-package "ROS")
;;//! \htmlinclude ConePairArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass racecar_cone_msgs::ConePairArray
  :super ros::object
  :slots (_header _pairs ))

(defmethod racecar_cone_msgs::ConePairArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pairs __pairs) (let (r) (dotimes (i 0) (push (instance racecar_cone_msgs::ConePair :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _pairs __pairs)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pairs
   (&rest __pairs)
   (if (keywordp (car __pairs))
       (send* _pairs __pairs)
     (progn
       (if __pairs (setq _pairs (car __pairs)))
       _pairs)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; racecar_cone_msgs/ConePair[] _pairs
    (apply #'+ (send-all _pairs :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; racecar_cone_msgs/ConePair[] _pairs
     (write-long (length _pairs) s)
     (dolist (elem _pairs)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; racecar_cone_msgs/ConePair[] _pairs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pairs (let (r) (dotimes (i n) (push (instance racecar_cone_msgs::ConePair :init) r)) r))
     (dolist (elem- _pairs)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get racecar_cone_msgs::ConePairArray :md5sum-) "fbdccabe23e08b5ded654d3910450cdc")
(setf (get racecar_cone_msgs::ConePairArray :datatype-) "racecar_cone_msgs/ConePairArray")
(setf (get racecar_cone_msgs::ConePairArray :definition-)
      "std_msgs/Header header
racecar_cone_msgs/ConePair[] pairs

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
MSG: racecar_cone_msgs/ConePair
std_msgs/Header header
racecar_cone_msgs/Cone left
racecar_cone_msgs/Cone right
geometry_msgs/Point midpoint
float32 width                   # ||right - left||

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



(provide :racecar_cone_msgs/ConePairArray "fbdccabe23e08b5ded654d3910450cdc")


