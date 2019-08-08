;; Auto-generated. Do not edit!


(when (boundp 'samana_msgs::Sonic9)
  (if (not (find-package "SAMANA_MSGS"))
    (make-package "SAMANA_MSGS"))
  (shadow 'Sonic9 (find-package "SAMANA_MSGS")))
(unless (find-package "SAMANA_MSGS::SONIC9")
  (make-package "SAMANA_MSGS::SONIC9"))

(in-package "ROS")
;;//! \htmlinclude Sonic9.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass samana_msgs::Sonic9
  :super ros::object
  :slots (_Header _dist1 _dist2 _dist3 _dist4 _dist5 _dist6 _dist7 _dist8 _dist9 ))

(defmethod samana_msgs::Sonic9
  (:init
   (&key
    ((:Header __Header) (instance std_msgs::Header :init))
    ((:dist1 __dist1) 0)
    ((:dist2 __dist2) 0)
    ((:dist3 __dist3) 0)
    ((:dist4 __dist4) 0)
    ((:dist5 __dist5) 0)
    ((:dist6 __dist6) 0)
    ((:dist7 __dist7) 0)
    ((:dist8 __dist8) 0)
    ((:dist9 __dist9) 0)
    )
   (send-super :init)
   (setq _Header __Header)
   (setq _dist1 (round __dist1))
   (setq _dist2 (round __dist2))
   (setq _dist3 (round __dist3))
   (setq _dist4 (round __dist4))
   (setq _dist5 (round __dist5))
   (setq _dist6 (round __dist6))
   (setq _dist7 (round __dist7))
   (setq _dist8 (round __dist8))
   (setq _dist9 (round __dist9))
   self)
  (:Header
   (&rest __Header)
   (if (keywordp (car __Header))
       (send* _Header __Header)
     (progn
       (if __Header (setq _Header (car __Header)))
       _Header)))
  (:dist1
   (&optional __dist1)
   (if __dist1 (setq _dist1 __dist1)) _dist1)
  (:dist2
   (&optional __dist2)
   (if __dist2 (setq _dist2 __dist2)) _dist2)
  (:dist3
   (&optional __dist3)
   (if __dist3 (setq _dist3 __dist3)) _dist3)
  (:dist4
   (&optional __dist4)
   (if __dist4 (setq _dist4 __dist4)) _dist4)
  (:dist5
   (&optional __dist5)
   (if __dist5 (setq _dist5 __dist5)) _dist5)
  (:dist6
   (&optional __dist6)
   (if __dist6 (setq _dist6 __dist6)) _dist6)
  (:dist7
   (&optional __dist7)
   (if __dist7 (setq _dist7 __dist7)) _dist7)
  (:dist8
   (&optional __dist8)
   (if __dist8 (setq _dist8 __dist8)) _dist8)
  (:dist9
   (&optional __dist9)
   (if __dist9 (setq _dist9 __dist9)) _dist9)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _Header
    (send _Header :serialization-length)
    ;; int16 _dist1
    2
    ;; int16 _dist2
    2
    ;; int16 _dist3
    2
    ;; int16 _dist4
    2
    ;; int16 _dist5
    2
    ;; int16 _dist6
    2
    ;; int16 _dist7
    2
    ;; int16 _dist8
    2
    ;; int16 _dist9
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _Header
       (send _Header :serialize s)
     ;; int16 _dist1
       (write-word _dist1 s)
     ;; int16 _dist2
       (write-word _dist2 s)
     ;; int16 _dist3
       (write-word _dist3 s)
     ;; int16 _dist4
       (write-word _dist4 s)
     ;; int16 _dist5
       (write-word _dist5 s)
     ;; int16 _dist6
       (write-word _dist6 s)
     ;; int16 _dist7
       (write-word _dist7 s)
     ;; int16 _dist8
       (write-word _dist8 s)
     ;; int16 _dist9
       (write-word _dist9 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _Header
     (send _Header :deserialize buf ptr-) (incf ptr- (send _Header :serialization-length))
   ;; int16 _dist1
     (setq _dist1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _dist2
     (setq _dist2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _dist3
     (setq _dist3 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _dist4
     (setq _dist4 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _dist5
     (setq _dist5 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _dist6
     (setq _dist6 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _dist7
     (setq _dist7 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _dist8
     (setq _dist8 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _dist9
     (setq _dist9 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get samana_msgs::Sonic9 :md5sum-) "9475c4b12aca5b9b235ea1a9e9e22280")
(setf (get samana_msgs::Sonic9 :datatype-) "samana_msgs/Sonic9")
(setf (get samana_msgs::Sonic9 :definition-)
      "std_msgs/Header Header
int16 dist1
int16 dist2
int16 dist3
int16 dist4
int16 dist5
int16 dist6
int16 dist7
int16 dist8
int16 dist9
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



(provide :samana_msgs/Sonic9 "9475c4b12aca5b9b235ea1a9e9e22280")


