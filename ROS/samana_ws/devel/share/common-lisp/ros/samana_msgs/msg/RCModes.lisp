; Auto-generated. Do not edit!


(cl:in-package samana_msgs-msg)


;//! \htmlinclude RCModes.msg.html

(cl:defclass <RCModes> (roslisp-msg-protocol:ros-message)
  ((allow_rc
    :reader allow_rc
    :initarg :allow_rc
    :type cl:boolean
    :initform cl:nil)
   (armed
    :reader armed
    :initarg :armed
    :type cl:boolean
    :initform cl:nil)
   (auton_mode
    :reader auton_mode
    :initarg :auton_mode
    :type cl:boolean
    :initform cl:nil)
   (arm_mode
    :reader arm_mode
    :initarg :arm_mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RCModes (<RCModes>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RCModes>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RCModes)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name samana_msgs-msg:<RCModes> is deprecated: use samana_msgs-msg:RCModes instead.")))

(cl:ensure-generic-function 'allow_rc-val :lambda-list '(m))
(cl:defmethod allow_rc-val ((m <RCModes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:allow_rc-val is deprecated.  Use samana_msgs-msg:allow_rc instead.")
  (allow_rc m))

(cl:ensure-generic-function 'armed-val :lambda-list '(m))
(cl:defmethod armed-val ((m <RCModes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:armed-val is deprecated.  Use samana_msgs-msg:armed instead.")
  (armed m))

(cl:ensure-generic-function 'auton_mode-val :lambda-list '(m))
(cl:defmethod auton_mode-val ((m <RCModes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:auton_mode-val is deprecated.  Use samana_msgs-msg:auton_mode instead.")
  (auton_mode m))

(cl:ensure-generic-function 'arm_mode-val :lambda-list '(m))
(cl:defmethod arm_mode-val ((m <RCModes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:arm_mode-val is deprecated.  Use samana_msgs-msg:arm_mode instead.")
  (arm_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RCModes>) ostream)
  "Serializes a message object of type '<RCModes>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'allow_rc) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'armed) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'auton_mode) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RCModes>) istream)
  "Deserializes a message object of type '<RCModes>"
    (cl:setf (cl:slot-value msg 'allow_rc) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'armed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'auton_mode) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RCModes>)))
  "Returns string type for a message object of type '<RCModes>"
  "samana_msgs/RCModes")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RCModes)))
  "Returns string type for a message object of type 'RCModes"
  "samana_msgs/RCModes")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RCModes>)))
  "Returns md5sum for a message object of type '<RCModes>"
  "f93b20a9c6f7b0344addef9c5a6daf0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RCModes)))
  "Returns md5sum for a message object of type 'RCModes"
  "f93b20a9c6f7b0344addef9c5a6daf0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RCModes>)))
  "Returns full string definition for message of type '<RCModes>"
  (cl:format cl:nil "bool allow_rc~%bool armed~%bool auton_mode~%uint8 arm_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RCModes)))
  "Returns full string definition for message of type 'RCModes"
  (cl:format cl:nil "bool allow_rc~%bool armed~%bool auton_mode~%uint8 arm_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RCModes>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RCModes>))
  "Converts a ROS message object to a list"
  (cl:list 'RCModes
    (cl:cons ':allow_rc (allow_rc msg))
    (cl:cons ':armed (armed msg))
    (cl:cons ':auton_mode (auton_mode msg))
    (cl:cons ':arm_mode (arm_mode msg))
))
