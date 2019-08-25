; Auto-generated. Do not edit!


(cl:in-package samana_msgs-msg)


;//! \htmlinclude Teleop.msg.html

(cl:defclass <Teleop> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0)
   (steer
    :reader steer
    :initarg :steer
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Teleop (<Teleop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Teleop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Teleop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name samana_msgs-msg:<Teleop> is deprecated: use samana_msgs-msg:Teleop instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Teleop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:speed-val is deprecated.  Use samana_msgs-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'steer-val :lambda-list '(m))
(cl:defmethod steer-val ((m <Teleop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:steer-val is deprecated.  Use samana_msgs-msg:steer instead.")
  (steer m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Teleop>) ostream)
  "Serializes a message object of type '<Teleop>"
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'steer)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Teleop>) istream)
  "Deserializes a message object of type '<Teleop>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'steer) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Teleop>)))
  "Returns string type for a message object of type '<Teleop>"
  "samana_msgs/Teleop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Teleop)))
  "Returns string type for a message object of type 'Teleop"
  "samana_msgs/Teleop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Teleop>)))
  "Returns md5sum for a message object of type '<Teleop>"
  "d8b9ad615ef7ce8cbee2931ec476027b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Teleop)))
  "Returns md5sum for a message object of type 'Teleop"
  "d8b9ad615ef7ce8cbee2931ec476027b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Teleop>)))
  "Returns full string definition for message of type '<Teleop>"
  (cl:format cl:nil "int16 speed~%int16 steer~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Teleop)))
  "Returns full string definition for message of type 'Teleop"
  (cl:format cl:nil "int16 speed~%int16 steer~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Teleop>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Teleop>))
  "Converts a ROS message object to a list"
  (cl:list 'Teleop
    (cl:cons ':speed (speed msg))
    (cl:cons ':steer (steer msg))
))
