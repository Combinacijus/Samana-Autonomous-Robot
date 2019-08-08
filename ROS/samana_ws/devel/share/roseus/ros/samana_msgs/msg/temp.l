;; Auto-generated. Do not edit!


(when (boundp 'samana_msgs::temp)
  (if (not (find-package "SAMANA_MSGS"))
    (make-package "SAMANA_MSGS"))
  (shadow 'temp (find-package "SAMANA_MSGS")))
(unless (find-package "SAMANA_MSGS::TEMP")
  (make-package "SAMANA_MSGS::TEMP"))

(in-package "ROS")
;;//! \htmlinclude temp.msg.html


(defclass samana_msgs::temp
  :super ros::object
  :slots (_temp ))

(defmethod samana_msgs::temp
  (:init
   (&key
    ((:temp __temp) 0.0)
    )
   (send-super :init)
   (setq _temp (float __temp))
   self)
  (:temp
   (&optional __temp)
   (if __temp (setq _temp __temp)) _temp)
  (:serialization-length
   ()
   (+
    ;; float32 _temp
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _temp
       (sys::poke _temp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _temp
     (setq _temp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get samana_msgs::temp :md5sum-) "b8b64af59df1604df7e647a04e645a45")
(setf (get samana_msgs::temp :datatype-) "samana_msgs/temp")
(setf (get samana_msgs::temp :definition-)
      "float32 temp
")



(provide :samana_msgs/temp "b8b64af59df1604df7e647a04e645a45")


