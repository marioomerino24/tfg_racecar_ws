;; Auto-generated. Do not edit!


(when (boundp 'racecar_cone_msgs::TrackMetrics)
  (if (not (find-package "RACECAR_CONE_MSGS"))
    (make-package "RACECAR_CONE_MSGS"))
  (shadow 'TrackMetrics (find-package "RACECAR_CONE_MSGS")))
(unless (find-package "RACECAR_CONE_MSGS::TRACKMETRICS")
  (make-package "RACECAR_CONE_MSGS::TRACKMETRICS"))

(in-package "ROS")
;;//! \htmlinclude TrackMetrics.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass racecar_cone_msgs::TrackMetrics
  :super ros::object
  :slots (_header _coverage _std_width _max_kappa _integral_kappa_sq _latency_ms _cones_raw_count _pairs_count _midpoints_count _notes ))

(defmethod racecar_cone_msgs::TrackMetrics
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:coverage __coverage) 0.0)
    ((:std_width __std_width) 0.0)
    ((:max_kappa __max_kappa) 0.0)
    ((:integral_kappa_sq __integral_kappa_sq) 0.0)
    ((:latency_ms __latency_ms) 0.0)
    ((:cones_raw_count __cones_raw_count) 0)
    ((:pairs_count __pairs_count) 0)
    ((:midpoints_count __midpoints_count) 0)
    ((:notes __notes) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _coverage (float __coverage))
   (setq _std_width (float __std_width))
   (setq _max_kappa (float __max_kappa))
   (setq _integral_kappa_sq (float __integral_kappa_sq))
   (setq _latency_ms (float __latency_ms))
   (setq _cones_raw_count (round __cones_raw_count))
   (setq _pairs_count (round __pairs_count))
   (setq _midpoints_count (round __midpoints_count))
   (setq _notes (string __notes))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:coverage
   (&optional __coverage)
   (if __coverage (setq _coverage __coverage)) _coverage)
  (:std_width
   (&optional __std_width)
   (if __std_width (setq _std_width __std_width)) _std_width)
  (:max_kappa
   (&optional __max_kappa)
   (if __max_kappa (setq _max_kappa __max_kappa)) _max_kappa)
  (:integral_kappa_sq
   (&optional __integral_kappa_sq)
   (if __integral_kappa_sq (setq _integral_kappa_sq __integral_kappa_sq)) _integral_kappa_sq)
  (:latency_ms
   (&optional __latency_ms)
   (if __latency_ms (setq _latency_ms __latency_ms)) _latency_ms)
  (:cones_raw_count
   (&optional __cones_raw_count)
   (if __cones_raw_count (setq _cones_raw_count __cones_raw_count)) _cones_raw_count)
  (:pairs_count
   (&optional __pairs_count)
   (if __pairs_count (setq _pairs_count __pairs_count)) _pairs_count)
  (:midpoints_count
   (&optional __midpoints_count)
   (if __midpoints_count (setq _midpoints_count __midpoints_count)) _midpoints_count)
  (:notes
   (&optional __notes)
   (if __notes (setq _notes __notes)) _notes)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _coverage
    4
    ;; float32 _std_width
    4
    ;; float32 _max_kappa
    4
    ;; float32 _integral_kappa_sq
    4
    ;; float32 _latency_ms
    4
    ;; uint32 _cones_raw_count
    4
    ;; uint32 _pairs_count
    4
    ;; uint32 _midpoints_count
    4
    ;; string _notes
    4 (length _notes)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _coverage
       (sys::poke _coverage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _std_width
       (sys::poke _std_width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_kappa
       (sys::poke _max_kappa (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _integral_kappa_sq
       (sys::poke _integral_kappa_sq (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _latency_ms
       (sys::poke _latency_ms (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _cones_raw_count
       (write-long _cones_raw_count s)
     ;; uint32 _pairs_count
       (write-long _pairs_count s)
     ;; uint32 _midpoints_count
       (write-long _midpoints_count s)
     ;; string _notes
       (write-long (length _notes) s) (princ _notes s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _coverage
     (setq _coverage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _std_width
     (setq _std_width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_kappa
     (setq _max_kappa (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _integral_kappa_sq
     (setq _integral_kappa_sq (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _latency_ms
     (setq _latency_ms (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _cones_raw_count
     (setq _cones_raw_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _pairs_count
     (setq _pairs_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _midpoints_count
     (setq _midpoints_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _notes
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _notes (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get racecar_cone_msgs::TrackMetrics :md5sum-) "9f81acf08fbcb042f5be8a05aaee3bf6")
(setf (get racecar_cone_msgs::TrackMetrics :datatype-) "racecar_cone_msgs/TrackMetrics")
(setf (get racecar_cone_msgs::TrackMetrics :definition-)
      "std_msgs/Header header

# Cobertura de emparejado (0..1)
float32 coverage

# Estadística de anchos
float32 std_width                      # desviación estándar de width (m)

# Curvatura y suavidad
float32 max_kappa                      # max |kappa| (1/m)
float32 integral_kappa_sq              # ∫ kappa^2 ds (1/m)

# Rendimiento
float32 latency_ms                     # latencia media del ciclo (ms)

# Contadores
uint32 cones_raw_count
uint32 pairs_count
uint32 midpoints_count

# Notas/diagnóstico
string notes

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

")



(provide :racecar_cone_msgs/TrackMetrics "9f81acf08fbcb042f5be8a05aaee3bf6")


