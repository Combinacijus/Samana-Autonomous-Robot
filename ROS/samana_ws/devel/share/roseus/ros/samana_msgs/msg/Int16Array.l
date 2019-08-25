;; Auto-generated. Do not edit!


(when (boundp 'samana_msgs::Int16Array)
  (if (not (find-package "SAMANA_MSGS"))
    (make-package "SAMANA_MSGS"))
  (shadow 'Int16Array (find-package "SAMANA_MSGS")))
(unless (find-package "SAMANA_MSGS::INT16ARRAY")
  (make-package "SAMANA_MSGS::INT16ARRAY"))

(in-package "ROS")
;;//! \htmlinclude Int16Array.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass samana_msgs::Int16Array
  :super ros::object
  :slots (_header _data ))

(defmethod samana_msgs::Int16Array
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16[] _data
    (* 2    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-word (elt _data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get samana_msgs::Int16Array :md5sum-) "dc47eb145a11cbe02d6c8cb8248a4099")
(setf (get samana_msgs::Int16Array :datatype-) "samana_msgs/Int16Array")
(setf (get samana_msgs::Int16Array :definition-)
      "std_msgs/Header header
int16[] data
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



(provide :samana_msgs/Int16Array "dc47eb145a11cbe02d6c8cb8248a4099")


