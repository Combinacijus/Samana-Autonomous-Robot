;; Auto-generated. Do not edit!


(when (boundp 'samana_msgs::ImuSmall)
  (if (not (find-package "SAMANA_MSGS"))
    (make-package "SAMANA_MSGS"))
  (shadow 'ImuSmall (find-package "SAMANA_MSGS")))
(unless (find-package "SAMANA_MSGS::IMUSMALL")
  (make-package "SAMANA_MSGS::IMUSMALL"))

(in-package "ROS")
;;//! \htmlinclude ImuSmall.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass samana_msgs::ImuSmall
  :super ros::object
  :slots (_header _quaternion_x _quaternion_y _quaternion_z _quaternion_w _linear_acceleration_x _linear_acceleration_y _linear_acceleration_z _angular_velocity_x _angular_velocity_y _angular_velocity_z ))

(defmethod samana_msgs::ImuSmall
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:quaternion_x __quaternion_x) 0.0)
    ((:quaternion_y __quaternion_y) 0.0)
    ((:quaternion_z __quaternion_z) 0.0)
    ((:quaternion_w __quaternion_w) 0.0)
    ((:linear_acceleration_x __linear_acceleration_x) 0.0)
    ((:linear_acceleration_y __linear_acceleration_y) 0.0)
    ((:linear_acceleration_z __linear_acceleration_z) 0.0)
    ((:angular_velocity_x __angular_velocity_x) 0.0)
    ((:angular_velocity_y __angular_velocity_y) 0.0)
    ((:angular_velocity_z __angular_velocity_z) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _quaternion_x (float __quaternion_x))
   (setq _quaternion_y (float __quaternion_y))
   (setq _quaternion_z (float __quaternion_z))
   (setq _quaternion_w (float __quaternion_w))
   (setq _linear_acceleration_x (float __linear_acceleration_x))
   (setq _linear_acceleration_y (float __linear_acceleration_y))
   (setq _linear_acceleration_z (float __linear_acceleration_z))
   (setq _angular_velocity_x (float __angular_velocity_x))
   (setq _angular_velocity_y (float __angular_velocity_y))
   (setq _angular_velocity_z (float __angular_velocity_z))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:quaternion_x
   (&optional __quaternion_x)
   (if __quaternion_x (setq _quaternion_x __quaternion_x)) _quaternion_x)
  (:quaternion_y
   (&optional __quaternion_y)
   (if __quaternion_y (setq _quaternion_y __quaternion_y)) _quaternion_y)
  (:quaternion_z
   (&optional __quaternion_z)
   (if __quaternion_z (setq _quaternion_z __quaternion_z)) _quaternion_z)
  (:quaternion_w
   (&optional __quaternion_w)
   (if __quaternion_w (setq _quaternion_w __quaternion_w)) _quaternion_w)
  (:linear_acceleration_x
   (&optional __linear_acceleration_x)
   (if __linear_acceleration_x (setq _linear_acceleration_x __linear_acceleration_x)) _linear_acceleration_x)
  (:linear_acceleration_y
   (&optional __linear_acceleration_y)
   (if __linear_acceleration_y (setq _linear_acceleration_y __linear_acceleration_y)) _linear_acceleration_y)
  (:linear_acceleration_z
   (&optional __linear_acceleration_z)
   (if __linear_acceleration_z (setq _linear_acceleration_z __linear_acceleration_z)) _linear_acceleration_z)
  (:angular_velocity_x
   (&optional __angular_velocity_x)
   (if __angular_velocity_x (setq _angular_velocity_x __angular_velocity_x)) _angular_velocity_x)
  (:angular_velocity_y
   (&optional __angular_velocity_y)
   (if __angular_velocity_y (setq _angular_velocity_y __angular_velocity_y)) _angular_velocity_y)
  (:angular_velocity_z
   (&optional __angular_velocity_z)
   (if __angular_velocity_z (setq _angular_velocity_z __angular_velocity_z)) _angular_velocity_z)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _quaternion_x
    4
    ;; float32 _quaternion_y
    4
    ;; float32 _quaternion_z
    4
    ;; float32 _quaternion_w
    4
    ;; float32 _linear_acceleration_x
    4
    ;; float32 _linear_acceleration_y
    4
    ;; float32 _linear_acceleration_z
    4
    ;; float32 _angular_velocity_x
    4
    ;; float32 _angular_velocity_y
    4
    ;; float32 _angular_velocity_z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _quaternion_x
       (sys::poke _quaternion_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _quaternion_y
       (sys::poke _quaternion_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _quaternion_z
       (sys::poke _quaternion_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _quaternion_w
       (sys::poke _quaternion_w (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linear_acceleration_x
       (sys::poke _linear_acceleration_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linear_acceleration_y
       (sys::poke _linear_acceleration_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linear_acceleration_z
       (sys::poke _linear_acceleration_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular_velocity_x
       (sys::poke _angular_velocity_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular_velocity_y
       (sys::poke _angular_velocity_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular_velocity_z
       (sys::poke _angular_velocity_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _quaternion_x
     (setq _quaternion_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _quaternion_y
     (setq _quaternion_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _quaternion_z
     (setq _quaternion_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _quaternion_w
     (setq _quaternion_w (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linear_acceleration_x
     (setq _linear_acceleration_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linear_acceleration_y
     (setq _linear_acceleration_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linear_acceleration_z
     (setq _linear_acceleration_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular_velocity_x
     (setq _angular_velocity_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular_velocity_y
     (setq _angular_velocity_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular_velocity_z
     (setq _angular_velocity_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get samana_msgs::ImuSmall :md5sum-) "c052128fb9568718800fa0ba7071e271")
(setf (get samana_msgs::ImuSmall :datatype-) "samana_msgs/ImuSmall")
(setf (get samana_msgs::ImuSmall :definition-)
      "std_msgs/Header header
float32 quaternion_x
float32 quaternion_y
float32 quaternion_z
float32 quaternion_w
float32 linear_acceleration_x
float32 linear_acceleration_y
float32 linear_acceleration_z
float32 angular_velocity_x
float32 angular_velocity_y
float32 angular_velocity_z
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



(provide :samana_msgs/ImuSmall "c052128fb9568718800fa0ba7071e271")


