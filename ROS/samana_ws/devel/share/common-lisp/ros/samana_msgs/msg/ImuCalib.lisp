; Auto-generated. Do not edit!


(cl:in-package samana_msgs-msg)


;//! \htmlinclude ImuCalib.msg.html

(cl:defclass <ImuCalib> (roslisp-msg-protocol:ros-message)
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

(cl:defclass ImuCalib (<ImuCalib>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImuCalib>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImuCalib)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name samana_msgs-msg:<ImuCalib> is deprecated: use samana_msgs-msg:ImuCalib instead.")))

(cl:ensure-generic-function 'sys-val :lambda-list '(m))
(cl:defmethod sys-val ((m <ImuCalib>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:sys-val is deprecated.  Use samana_msgs-msg:sys instead.")
  (sys m))

(cl:ensure-generic-function 'gyr-val :lambda-list '(m))
(cl:defmethod gyr-val ((m <ImuCalib>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:gyr-val is deprecated.  Use samana_msgs-msg:gyr instead.")
  (gyr m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <ImuCalib>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:acc-val is deprecated.  Use samana_msgs-msg:acc instead.")
  (acc m))

(cl:ensure-generic-function 'mag-val :lambda-list '(m))
(cl:defmethod mag-val ((m <ImuCalib>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:mag-val is deprecated.  Use samana_msgs-msg:mag instead.")
  (mag m))

(cl:ensure-generic-function 'temp-val :lambda-list '(m))
(cl:defmethod temp-val ((m <ImuCalib>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:temp-val is deprecated.  Use samana_msgs-msg:temp instead.")
  (temp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImuCalib>) ostream)
  "Serializes a message object of type '<ImuCalib>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sys)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gyr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'acc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mag)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'temp)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImuCalib>) istream)
  "Deserializes a message object of type '<ImuCalib>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sys)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gyr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'acc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mag)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'temp) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImuCalib>)))
  "Returns string type for a message object of type '<ImuCalib>"
  "samana_msgs/ImuCalib")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImuCalib)))
  "Returns string type for a message object of type 'ImuCalib"
  "samana_msgs/ImuCalib")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImuCalib>)))
  "Returns md5sum for a message object of type '<ImuCalib>"
  "7764c0234b4e443d9ef754fa0119997d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImuCalib)))
  "Returns md5sum for a message object of type 'ImuCalib"
  "7764c0234b4e443d9ef754fa0119997d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImuCalib>)))
  "Returns full string definition for message of type '<ImuCalib>"
  (cl:format cl:nil "uint8 sys~%uint8 gyr~%uint8 acc~%uint8 mag~%int8 temp~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImuCalib)))
  "Returns full string definition for message of type 'ImuCalib"
  (cl:format cl:nil "uint8 sys~%uint8 gyr~%uint8 acc~%uint8 mag~%int8 temp~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImuCalib>))
  (cl:+ 0
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImuCalib>))
  "Converts a ROS message object to a list"
  (cl:list 'ImuCalib
    (cl:cons ':sys (sys msg))
    (cl:cons ':gyr (gyr msg))
    (cl:cons ':acc (acc msg))
    (cl:cons ':mag (mag msg))
    (cl:cons ':temp (temp msg))
))
