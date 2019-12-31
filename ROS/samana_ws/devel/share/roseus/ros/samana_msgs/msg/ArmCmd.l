;; Auto-generated. Do not edit!


(when (boundp 'samana_msgs::ArmCmd)
  (if (not (find-package "SAMANA_MSGS"))
    (make-package "SAMANA_MSGS"))
  (shadow 'ArmCmd (find-package "SAMANA_MSGS")))
(unless (find-package "SAMANA_MSGS::ARMCMD")
  (make-package "SAMANA_MSGS::ARMCMD"))

(in-package "ROS")
;;//! \htmlinclude ArmCmd.msg.html


(defclass samana_msgs::ArmCmd
  :super ros::object
  :slots (_grabber_cmd _lifter_cmd ))

(defmethod samana_msgs::ArmCmd
  (:init
   (&key
    ((:grabber_cmd __grabber_cmd) 0)
    ((:lifter_cmd __lifter_cmd) 0)
    )
   (send-super :init)
   (setq _grabber_cmd (round __grabber_cmd))
   (setq _lifter_cmd (round __lifter_cmd))
   self)
  (:grabber_cmd
   (&optional __grabber_cmd)
   (if __grabber_cmd (setq _grabber_cmd __grabber_cmd)) _grabber_cmd)
  (:lifter_cmd
   (&optional __lifter_cmd)
   (if __lifter_cmd (setq _lifter_cmd __lifter_cmd)) _lifter_cmd)
  (:serialization-length
   ()
   (+
    ;; int8 _grabber_cmd
    1
    ;; int8 _lifter_cmd
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _grabber_cmd
       (write-byte _grabber_cmd s)
     ;; int8 _lifter_cmd
       (write-byte _lifter_cmd s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _grabber_cmd
     (setq _grabber_cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _grabber_cmd 127) (setq _grabber_cmd (- _grabber_cmd 256)))
   ;; int8 _lifter_cmd
     (setq _lifter_cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _lifter_cmd 127) (setq _lifter_cmd (- _lifter_cmd 256)))
   ;;
   self)
  )

(setf (get samana_msgs::ArmCmd :md5sum-) "4915ef1fb595c9707da4cb79c7caeeb8")
(setf (get samana_msgs::ArmCmd :datatype-) "samana_msgs/ArmCmd")
(setf (get samana_msgs::ArmCmd :definition-)
      "int8 grabber_cmd
int8 lifter_cmd
")



(provide :samana_msgs/ArmCmd "4915ef1fb595c9707da4cb79c7caeeb8")


