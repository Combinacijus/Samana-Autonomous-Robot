;; Auto-generated. Do not edit!


(when (boundp 'samana_msgs::OdometrySmall)
  (if (not (find-package "SAMANA_MSGS"))
    (make-package "SAMANA_MSGS"))
  (shadow 'OdometrySmall (find-package "SAMANA_MSGS")))
(unless (find-package "SAMANA_MSGS::ODOMETRYSMALL")
  (make-package "SAMANA_MSGS::ODOMETRYSMALL"))

(in-package "ROS")
;;//! \htmlinclude OdometrySmall.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass samana_msgs::OdometrySmall
  :super ros::object
  :slots (_header _delta_ticks1 _delta_ticks2 _rps1 _rps2 _dt ))

(defmethod samana_msgs::OdometrySmall
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:delta_ticks1 __delta_ticks1) 0)
    ((:delta_ticks2 __delta_ticks2) 0)
    ((:rps1 __rps1) 0.0)
    ((:rps2 __rps2) 0.0)
    ((:dt __dt) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _delta_ticks1 (round __delta_ticks1))
   (setq _delta_ticks2 (round __delta_ticks2))
   (setq _rps1 (float __rps1))
   (setq _rps2 (float __rps2))
   (setq _dt (round __dt))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:delta_ticks1
   (&optional __delta_ticks1)
   (if __delta_ticks1 (setq _delta_ticks1 __delta_ticks1)) _delta_ticks1)
  (:delta_ticks2
   (&optional __delta_ticks2)
   (if __delta_ticks2 (setq _delta_ticks2 __delta_ticks2)) _delta_ticks2)
  (:rps1
   (&optional __rps1)
   (if __rps1 (setq _rps1 __rps1)) _rps1)
  (:rps2
   (&optional __rps2)
   (if __rps2 (setq _rps2 __rps2)) _rps2)
  (:dt
   (&optional __dt)
   (if __dt (setq _dt __dt)) _dt)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16 _delta_ticks1
    2
    ;; int16 _delta_ticks2
    2
    ;; float32 _rps1
    4
    ;; float32 _rps2
    4
    ;; int16 _dt
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16 _delta_ticks1
       (write-word _delta_ticks1 s)
     ;; int16 _delta_ticks2
       (write-word _delta_ticks2 s)
     ;; float32 _rps1
       (sys::poke _rps1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rps2
       (sys::poke _rps2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int16 _dt
       (write-word _dt s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16 _delta_ticks1
     (setq _delta_ticks1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _delta_ticks2
     (setq _delta_ticks2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _rps1
     (setq _rps1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rps2
     (setq _rps2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int16 _dt
     (setq _dt (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get samana_msgs::OdometrySmall :md5sum-) "98004b29be7f03ed0afbf8488b6e7875")
(setf (get samana_msgs::OdometrySmall :datatype-) "samana_msgs/OdometrySmall")
(setf (get samana_msgs::OdometrySmall :definition-)
      "std_msgs/Header header
int16 delta_ticks1
int16 delta_ticks2
float32 rps1
float32 rps2
int16 dt
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



(provide :samana_msgs/OdometrySmall "98004b29be7f03ed0afbf8488b6e7875")


