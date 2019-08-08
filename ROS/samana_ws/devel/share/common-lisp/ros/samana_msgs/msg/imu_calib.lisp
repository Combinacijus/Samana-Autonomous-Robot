; Auto-generated. Do not edit!


(cl:in-package samana_msgs-msg)


;//! \htmlinclude imu_calib.msg.html

(cl:defclass <imu_calib> (roslisp-msg-protocol:ros-message)
  ((sys
    :reader sys
    :initarg :sys
    :type cl:fixnum
    :initform 0)
   (gyr
    :reader gyr
    :initarg :gyr
    :type cl:fixnum
    :initform 0)
   (acc
    :reader acc
    :initarg :acc
    :type cl:fixnum
    :initform 0)
   (mag
    :reader mag
    :initarg :mag
    :type cl:fixnum
    :initform 0)
   (temp
    :reader temp
    :initarg :temp
    :type cl:fixnum
    :initform 0))
)

(cl:defclass imu_calib (<imu_calib>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <imu_calib>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'imu_calib)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name samana_msgs-msg:<imu_calib> is deprecated: use samana_msgs-msg:imu_calib instead.")))

(cl:ensure-generic-function 'sys-val :lambda-list '(m))
(cl:defmethod sys-val ((m <imu_calib>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:sys-val is deprecated.  Use samana_msgs-msg:sys instead.")
  (sys m))

(cl:ensure-generic-function 'gyr-val :lambda-list '(m))
(cl:defmethod gyr-val ((m <imu_calib>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:gyr-val is deprecated.  Use samana_msgs-msg:gyr instead.")
  (gyr m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <imu_calib>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:acc-val is deprecated.  Use samana_msgs-msg:acc instead.")
  (acc m))

(cl:ensure-generic-function 'mag-val :lambda-list '(m))
(cl:defmethod mag-val ((m <imu_calib>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:mag-val is deprecated.  Use samana_msgs-msg:mag instead.")
  (mag m))

(cl:ensure-generic-function 'temp-val :lambda-list '(m))
(cl:defmethod temp-val ((m <imu_calib>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:temp-val is deprecated.  Use samana_msgs-msg:temp instead.")
  (temp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <imu_calib>) ostream)
  "Serializes a message object of type '<imu_calib>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sys)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gyr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'acc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mag)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'temp)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <imu_calib>) istream)
  "Deserializes a message object of type '<imu_calib>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sys)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gyr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'acc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mag)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'temp) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<imu_calib>)))
  "Returns string type for a message object of type '<imu_calib>"
  "samana_msgs/imu_calib")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'imu_calib)))
  "Returns string type for a message object of type 'imu_calib"
  "samana_msgs/imu_calib")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<imu_calib>)))
  "Returns md5sum for a message object of type '<imu_calib>"
  "7764c0234b4e443d9ef754fa0119997d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'imu_calib)))
  "Returns md5sum for a message object of type 'imu_calib"
  "7764c0234b4e443d9ef754fa0119997d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<imu_calib>)))
  "Returns full string definition for message of type '<imu_calib>"
  (cl:format cl:nil "uint8 sys~%uint8 gyr~%uint8 acc~%uint8 mag~%int8 temp~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'imu_calib)))
  "Returns full string definition for message of type 'imu_calib"
  (cl:format cl:nil "uint8 sys~%uint8 gyr~%uint8 acc~%uint8 mag~%int8 temp~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <imu_calib>))
  (cl:+ 0
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <imu_calib>))
  "Converts a ROS message object to a list"
  (cl:list 'imu_calib
    (cl:cons ':sys (sys msg))
    (cl:cons ':gyr (gyr msg))
    (cl:cons ':acc (acc msg))
    (cl:cons ':mag (mag msg))
    (cl:cons ':temp (temp msg))
))
