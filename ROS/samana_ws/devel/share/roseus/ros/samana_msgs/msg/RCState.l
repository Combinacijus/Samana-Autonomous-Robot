;; Auto-generated. Do not edit!


(when (boundp 'samana_msgs::RCState)
  (if (not (find-package "SAMANA_MSGS"))
    (make-package "SAMANA_MSGS"))
  (shadow 'RCState (find-package "SAMANA_MSGS")))
(unless (find-package "SAMANA_MSGS::RCSTATE")
  (make-package "SAMANA_MSGS::RCSTATE"))

(in-package "ROS")
;;//! \htmlinclude RCState.msg.html


(defclass samana_msgs::RCState
  :super ros::object
  :slots (_allow_rc _armed _auton_mode _arm_mode ))

(defmethod samana_msgs::RCState
  (:init
   (&key
    ((:allow_rc __allow_rc) nil)
    ((:armed __armed) nil)
    ((:auton_mode __auton_mode) nil)
    ((:arm_mode __arm_mode) 0)
    )
   (send-super :init)
   (setq _allow_rc __allow_rc)
   (setq _armed __armed)
   (setq _auton_mode __auton_mode)
   (setq _arm_mode (round __arm_mode))
   self)
  (:allow_rc
   (&optional __allow_rc)
   (if __allow_rc (setq _allow_rc __allow_rc)) _allow_rc)
  (:armed
   (&optional __armed)
   (if __armed (setq _armed __armed)) _armed)
  (:auton_mode
   (&optional __auton_mode)
   (if __auton_mode (setq _auton_mode __auton_mode)) _auton_mode)
  (:arm_mode
   (&optional __arm_mode)
   (if __arm_mode (setq _arm_mode __arm_mode)) _arm_mode)
  (:serialization-length
   ()
   (+
    ;; bool _allow_rc
    1
    ;; bool _armed
    1
    ;; bool _auton_mode
    1
    ;; uint8 _arm_mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _allow_rc
       (if _allow_rc (write-byte -1 s) (write-byte 0 s))
     ;; bool _armed
       (if _armed (write-byte -1 s) (write-byte 0 s))
     ;; bool _auton_mode
       (if _auton_mode (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _arm_mode
       (write-byte _arm_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _allow_rc
     (setq _allow_rc (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _armed
     (setq _armed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _auton_mode
     (setq _auton_mode (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _arm_mode
     (setq _arm_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get samana_msgs::RCState :md5sum-) "f93b20a9c6f7b0344addef9c5a6daf0f")
(setf (get samana_msgs::RCState :datatype-) "samana_msgs/RCState")
(setf (get samana_msgs::RCState :definition-)
      "bool allow_rc
bool armed
bool auton_mode
uint8 arm_mode
")



(provide :samana_msgs/RCState "f93b20a9c6f7b0344addef9c5a6daf0f")


