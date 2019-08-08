;; Auto-generated. Do not edit!


(when (boundp 'samana_msgs::Sonar)
  (if (not (find-package "SAMANA_MSGS"))
    (make-package "SAMANA_MSGS"))
  (shadow 'Sonar (find-package "SAMANA_MSGS")))
(unless (find-package "SAMANA_MSGS::SONAR")
  (make-package "SAMANA_MSGS::SONAR"))

(in-package "ROS")
;;//! \htmlinclude Sonar.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass samana_msgs::Sonar
  :super ros::object
  :slots (_header _dist ))

(defmethod samana_msgs::Sonar
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:dist __dist) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _dist __dist)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:dist
   (&optional __dist)
   (if __dist (setq _dist __dist)) _dist)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16[] _dist
    (* 2    (length _dist)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16[] _dist
     (write-long (length _dist) s)
     (dotimes (i (length _dist))
       (write-word (elt _dist i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16[] _dist
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dist (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _dist i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get samana_msgs::Sonar :md5sum-) "ff005e0bae1d9bc418c95faadd17f36d")
(setf (get samana_msgs::Sonar :datatype-) "samana_msgs/Sonar")
(setf (get samana_msgs::Sonar :definition-)
      "std_msgs/Header header
int16[] dist
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



(provide :samana_msgs/Sonar "ff005e0bae1d9bc418c95faadd17f36d")


