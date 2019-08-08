;; Auto-generated. Do not edit!


(when (boundp 'samana_msgs::ImuCalib)
  (if (not (find-package "SAMANA_MSGS"))
    (make-package "SAMANA_MSGS"))
  (shadow 'ImuCalib (find-package "SAMANA_MSGS")))
(unless (find-package "SAMANA_MSGS::IMUCALIB")
  (make-package "SAMANA_MSGS::IMUCALIB"))

(in-package "ROS")
;;//! \htmlinclude ImuCalib.msg.html


(defclass samana_msgs::ImuCalib
  :super ros::object
  :slots (_sys _gyr _acc _mag _temp ))

(defmethod samana_msgs::ImuCalib
  (:init
   (&key
    ((:sys __sys) 0)
    ((:gyr __gyr) 0)
    ((:acc __acc) 0)
    ((:mag __mag) 0)
    ((:temp __temp) 0)
    )
   (send-super :init)
   (setq _sys (round __sys))
   (setq _gyr (round __gyr))
   (setq _acc (round __acc))
   (setq _mag (round __mag))
   (setq _temp (round __temp))
   self)
  (:sys
   (&optional __sys)
   (if __sys (setq _sys __sys)) _sys)
  (:gyr
   (&optional __gyr)
   (if __gyr (setq _gyr __gyr)) _gyr)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:mag
   (&optional __mag)
   (if __mag (setq _mag __mag)) _mag)
  (:temp
   (&optional __temp)
   (if __temp (setq _temp __temp)) _temp)
  (:serialization-length
   ()
   (+
    ;; uint8 _sys
    1
    ;; uint8 _gyr
    1
    ;; uint8 _acc
    1
    ;; uint8 _mag
    1
    ;; int8 _temp
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _sys
       (write-byte _sys s)
     ;; uint8 _gyr
       (write-byte _gyr s)
     ;; uint8 _acc
       (write-byte _acc s)
     ;; uint8 _mag
       (write-byte _mag s)
     ;; int8 _temp
       (write-byte _temp s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _sys
     (setq _sys (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gyr
     (setq _gyr (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _acc
     (setq _acc (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _mag
     (setq _mag (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int8 _temp
     (setq _temp (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _temp 127) (setq _temp (- _temp 256)))
   ;;
   self)
  )

(setf (get samana_msgs::ImuCalib :md5sum-) "7764c0234b4e443d9ef754fa0119997d")
(setf (get samana_msgs::ImuCalib :datatype-) "samana_msgs/ImuCalib")
(setf (get samana_msgs::ImuCalib :definition-)
      "uint8 sys
uint8 gyr
uint8 acc
uint8 mag
int8 temp
")



(provide :samana_msgs/ImuCalib "7764c0234b4e443d9ef754fa0119997d")


