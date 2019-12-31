; Auto-generated. Do not edit!


(cl:in-package samana_msgs-msg)


;//! \htmlinclude ArmData.msg.html

(cl:defclass <ArmData> (roslisp-msg-protocol:ros-message)
  ((current_grabber
    :reader current_grabber
    :initarg :current_grabber
    :type cl:fixnum
    :initform 0)
   (current_lifter
    :reader current_lifter
    :initarg :current_lifter
    :type cl:fixnum
    :initform 0)
   (limit_switches
    :reader limit_switches
    :initarg :limit_switches
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ArmData (<ArmData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name samana_msgs-msg:<ArmData> is deprecated: use samana_msgs-msg:ArmData instead.")))

(cl:ensure-generic-function 'current_grabber-val :lambda-list '(m))
(cl:defmethod current_grabber-val ((m <ArmData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:current_grabber-val is deprecated.  Use samana_msgs-msg:current_grabber instead.")
  (current_grabber m))

(cl:ensure-generic-function 'current_lifter-val :lambda-list '(m))
(cl:defmethod current_lifter-val ((m <ArmData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:current_lifter-val is deprecated.  Use samana_msgs-msg:current_lifter instead.")
  (current_lifter m))

(cl:ensure-generic-function 'limit_switches-val :lambda-list '(m))
(cl:defmethod limit_switches-val ((m <ArmData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:limit_switches-val is deprecated.  Use samana_msgs-msg:limit_switches instead.")
  (limit_switches m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmData>) ostream)
  "Serializes a message object of type '<ArmData>"
  (cl:let* ((signed (cl:slot-value msg 'current_grabber)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'current_lifter)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'limit_switches)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmData>) istream)
  "Deserializes a message object of type '<ArmData>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_grabber) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_lifter) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'limit_switches)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmData>)))
  "Returns string type for a message object of type '<ArmData>"
  "samana_msgs/ArmData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmData)))
  "Returns string type for a message object of type 'ArmData"
  "samana_msgs/ArmData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmData>)))
  "Returns md5sum for a message object of type '<ArmData>"
  "3812456ac8efcba0c0c24a88991ba799")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmData)))
  "Returns md5sum for a message object of type 'ArmData"
  "3812456ac8efcba0c0c24a88991ba799")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmData>)))
  "Returns full string definition for message of type '<ArmData>"
  (cl:format cl:nil "int16 current_grabber~%int16 current_lifter~%uint8 limit_switches~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmData)))
  "Returns full string definition for message of type 'ArmData"
  (cl:format cl:nil "int16 current_grabber~%int16 current_lifter~%uint8 limit_switches~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmData>))
  (cl:+ 0
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmData>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmData
    (cl:cons ':current_grabber (current_grabber msg))
    (cl:cons ':current_lifter (current_lifter msg))
    (cl:cons ':limit_switches (limit_switches msg))
))
