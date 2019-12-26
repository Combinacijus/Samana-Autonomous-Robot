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
  :slots (_header _ticks1 _ticks2 _rps1 _rps2 ))

(defmethod samana_msgs::OdometrySmall
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ticks1 __ticks1) 0)
    ((:ticks2 __ticks2) 0)
    ((:rps1 __rps1) 0.0)
    ((:rps2 __rps2) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _ticks1 (round __ticks1))
   (setq _ticks2 (round __ticks2))
   (setq _rps1 (float __rps1))
   (setq _rps2 (float __rps2))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ticks1
   (&optional __ticks1)
   (if __ticks1 (setq _ticks1 __ticks1)) _ticks1)
  (:ticks2
   (&optional __ticks2)
   (if __ticks2 (setq _ticks2 __ticks2)) _ticks2)
  (:rps1
   (&optional __rps1)
   (if __rps1 (setq _rps1 __rps1)) _rps1)
  (:rps2
   (&optional __rps2)
   (if __rps2 (setq _rps2 __rps2)) _rps2)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _ticks1
    2
    ;; uint16 _ticks2
    2
    ;; float32 _rps1
    4
    ;; float32 _rps2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _ticks1
       (write-word _ticks1 s)
     ;; uint16 _ticks2
       (write-word _ticks2 s)
     ;; float32 _rps1
       (sys::poke _rps1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rps2
       (sys::poke _rps2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _ticks1
     (setq _ticks1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _ticks2
     (setq _ticks2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _rps1
     (setq _rps1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rps2
     (setq _rps2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get samana_msgs::OdometrySmall :md5sum-) "eac9f20217e8b1524f88ba55f5ff1284")
(setf (get samana_msgs::OdometrySmall :datatype-) "samana_msgs/OdometrySmall")
(setf (get samana_msgs::OdometrySmall :definition-)
      "std_msgs/Header header
uint16 ticks1
uint16 ticks2
float32 rps1
float32 rps2
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



(provide :samana_msgs/OdometrySmall "eac9f20217e8b1524f88ba55f5ff1284")


