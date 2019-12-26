;; Auto-generated. Do not edit!


(when (boundp 'samana_msgs::ArmData)
  (if (not (find-package "SAMANA_MSGS"))
    (make-package "SAMANA_MSGS"))
  (shadow 'ArmData (find-package "SAMANA_MSGS")))
(unless (find-package "SAMANA_MSGS::ARMDATA")
  (make-package "SAMANA_MSGS::ARMDATA"))

(in-package "ROS")
;;//! \htmlinclude ArmData.msg.html


(defclass samana_msgs::ArmData
  :super ros::object
  :slots (_current_grabber _current_lifter _limit_switches ))

(defmethod samana_msgs::ArmData
  (:init
   (&key
    ((:current_grabber __current_grabber) 0)
    ((:current_lifter __current_lifter) 0)
    ((:limit_switches __limit_switches) 0)
    )
   (send-super :init)
   (setq _current_grabber (round __current_grabber))
   (setq _current_lifter (round __current_lifter))
   (setq _limit_switches (round __limit_switches))
   self)
  (:current_grabber
   (&optional __current_grabber)
   (if __current_grabber (setq _current_grabber __current_grabber)) _current_grabber)
  (:current_lifter
   (&optional __current_lifter)
   (if __current_lifter (setq _current_lifter __current_lifter)) _current_lifter)
  (:limit_switches
   (&optional __limit_switches)
   (if __limit_switches (setq _limit_switches __limit_switches)) _limit_switches)
  (:serialization-length
   ()
   (+
    ;; int16 _current_grabber
    2
    ;; int16 _current_lifter
    2
    ;; uint8 _limit_switches
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _current_grabber
       (write-word _current_grabber s)
     ;; int16 _current_lifter
       (write-word _current_lifter s)
     ;; uint8 _limit_switches
       (write-byte _limit_switches s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _current_grabber
     (setq _current_grabber (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _current_lifter
     (setq _current_lifter (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _limit_switches
     (setq _limit_switches (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get samana_msgs::ArmData :md5sum-) "3812456ac8efcba0c0c24a88991ba799")
(setf (get samana_msgs::ArmData :datatype-) "samana_msgs/ArmData")
(setf (get samana_msgs::ArmData :definition-)
      "int16 current_grabber
int16 current_lifter
uint8 limit_switches

")



(provide :samana_msgs/ArmData "3812456ac8efcba0c0c24a88991ba799")


