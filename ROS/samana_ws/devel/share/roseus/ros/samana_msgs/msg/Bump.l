;; Auto-generated. Do not edit!


(when (boundp 'samana_msgs::Bump)
  (if (not (find-package "SAMANA_MSGS"))
    (make-package "SAMANA_MSGS"))
  (shadow 'Bump (find-package "SAMANA_MSGS")))
(unless (find-package "SAMANA_MSGS::BUMP")
  (make-package "SAMANA_MSGS::BUMP"))

(in-package "ROS")
;;//! \htmlinclude Bump.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass samana_msgs::Bump
  :super ros::object
  :slots (_header _bump_bits ))

(defmethod samana_msgs::Bump
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:bump_bits __bump_bits) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _bump_bits (round __bump_bits))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:bump_bits
   (&optional __bump_bits)
   (if __bump_bits (setq _bump_bits __bump_bits)) _bump_bits)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _bump_bits
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _bump_bits
       (write-word _bump_bits s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _bump_bits
     (setq _bump_bits (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get samana_msgs::Bump :md5sum-) "be760e4b4932adb10f1e9fc66f2d1769")
(setf (get samana_msgs::Bump :datatype-) "samana_msgs/Bump")
(setf (get samana_msgs::Bump :definition-)
      "std_msgs/Header header
uint16 bump_bits
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



(provide :samana_msgs/Bump "be760e4b4932adb10f1e9fc66f2d1769")


