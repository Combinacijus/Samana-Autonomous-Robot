;; Auto-generated. Do not edit!


(when (boundp 'samana_msgs::Teleop)
  (if (not (find-package "SAMANA_MSGS"))
    (make-package "SAMANA_MSGS"))
  (shadow 'Teleop (find-package "SAMANA_MSGS")))
(unless (find-package "SAMANA_MSGS::TELEOP")
  (make-package "SAMANA_MSGS::TELEOP"))

(in-package "ROS")
;;//! \htmlinclude Teleop.msg.html


(defclass samana_msgs::Teleop
  :super ros::object
  :slots (_speed _steer ))

(defmethod samana_msgs::Teleop
  (:init
   (&key
    ((:speed __speed) 0)
    ((:steer __steer) 0)
    )
   (send-super :init)
   (setq _speed (round __speed))
   (setq _steer (round __steer))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:steer
   (&optional __steer)
   (if __steer (setq _steer __steer)) _steer)
  (:serialization-length
   ()
   (+
    ;; int16 _speed
    2
    ;; int16 _steer
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _speed
       (write-word _speed s)
     ;; int16 _steer
       (write-word _steer s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _speed
     (setq _speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _steer
     (setq _steer (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get samana_msgs::Teleop :md5sum-) "d8b9ad615ef7ce8cbee2931ec476027b")
(setf (get samana_msgs::Teleop :datatype-) "samana_msgs/Teleop")
(setf (get samana_msgs::Teleop :definition-)
      "int16 speed
int16 steer
")



(provide :samana_msgs/Teleop "d8b9ad615ef7ce8cbee2931ec476027b")


