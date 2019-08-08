; Auto-generated. Do not edit!


(cl:in-package samana_msgs-msg)


;//! \htmlinclude ImuSmall.msg.html

(cl:defclass <ImuSmall> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (quaternion_x
    :reader quaternion_x
    :initarg :quaternion_x
    :type cl:float
    :initform 0.0)
   (quaternion_y
    :reader quaternion_y
    :initarg :quaternion_y
    :type cl:float
    :initform 0.0)
   (quaternion_z
    :reader quaternion_z
    :initarg :quaternion_z
    :type cl:float
    :initform 0.0)
   (quaternion_w
    :reader quaternion_w
    :initarg :quaternion_w
    :type cl:float
    :initform 0.0)
   (linear_acceleration_x
    :reader linear_acceleration_x
    :initarg :linear_acceleration_x
    :type cl:float
    :initform 0.0)
   (linear_acceleration_y
    :reader linear_acceleration_y
    :initarg :linear_acceleration_y
    :type cl:float
    :initform 0.0)
   (linear_acceleration_z
    :reader linear_acceleration_z
    :initarg :linear_acceleration_z
    :type cl:float
    :initform 0.0)
   (angular_velocity_x
    :reader angular_velocity_x
    :initarg :angular_velocity_x
    :type cl:float
    :initform 0.0)
   (angular_velocity_y
    :reader angular_velocity_y
    :initarg :angular_velocity_y
    :type cl:float
    :initform 0.0)
   (angular_velocity_z
    :reader angular_velocity_z
    :initarg :angular_velocity_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass ImuSmall (<ImuSmall>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImuSmall>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImuSmall)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name samana_msgs-msg:<ImuSmall> is deprecated: use samana_msgs-msg:ImuSmall instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ImuSmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:header-val is deprecated.  Use samana_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'quaternion_x-val :lambda-list '(m))
(cl:defmethod quaternion_x-val ((m <ImuSmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:quaternion_x-val is deprecated.  Use samana_msgs-msg:quaternion_x instead.")
  (quaternion_x m))

(cl:ensure-generic-function 'quaternion_y-val :lambda-list '(m))
(cl:defmethod quaternion_y-val ((m <ImuSmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:quaternion_y-val is deprecated.  Use samana_msgs-msg:quaternion_y instead.")
  (quaternion_y m))

(cl:ensure-generic-function 'quaternion_z-val :lambda-list '(m))
(cl:defmethod quaternion_z-val ((m <ImuSmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:quaternion_z-val is deprecated.  Use samana_msgs-msg:quaternion_z instead.")
  (quaternion_z m))

(cl:ensure-generic-function 'quaternion_w-val :lambda-list '(m))
(cl:defmethod quaternion_w-val ((m <ImuSmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:quaternion_w-val is deprecated.  Use samana_msgs-msg:quaternion_w instead.")
  (quaternion_w m))

(cl:ensure-generic-function 'linear_acceleration_x-val :lambda-list '(m))
(cl:defmethod linear_acceleration_x-val ((m <ImuSmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:linear_acceleration_x-val is deprecated.  Use samana_msgs-msg:linear_acceleration_x instead.")
  (linear_acceleration_x m))

(cl:ensure-generic-function 'linear_acceleration_y-val :lambda-list '(m))
(cl:defmethod linear_acceleration_y-val ((m <ImuSmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:linear_acceleration_y-val is deprecated.  Use samana_msgs-msg:linear_acceleration_y instead.")
  (linear_acceleration_y m))

(cl:ensure-generic-function 'linear_acceleration_z-val :lambda-list '(m))
(cl:defmethod linear_acceleration_z-val ((m <ImuSmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:linear_acceleration_z-val is deprecated.  Use samana_msgs-msg:linear_acceleration_z instead.")
  (linear_acceleration_z m))

(cl:ensure-generic-function 'angular_velocity_x-val :lambda-list '(m))
(cl:defmethod angular_velocity_x-val ((m <ImuSmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:angular_velocity_x-val is deprecated.  Use samana_msgs-msg:angular_velocity_x instead.")
  (angular_velocity_x m))

(cl:ensure-generic-function 'angular_velocity_y-val :lambda-list '(m))
(cl:defmethod angular_velocity_y-val ((m <ImuSmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:angular_velocity_y-val is deprecated.  Use samana_msgs-msg:angular_velocity_y instead.")
  (angular_velocity_y m))

(cl:ensure-generic-function 'angular_velocity_z-val :lambda-list '(m))
(cl:defmethod angular_velocity_z-val ((m <ImuSmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:angular_velocity_z-val is deprecated.  Use samana_msgs-msg:angular_velocity_z instead.")
  (angular_velocity_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImuSmall>) ostream)
  "Serializes a message object of type '<ImuSmall>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'quaternion_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'quaternion_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'quaternion_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'quaternion_w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linear_acceleration_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linear_acceleration_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linear_acceleration_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular_velocity_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular_velocity_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular_velocity_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImuSmall>) istream)
  "Deserializes a message object of type '<ImuSmall>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'quaternion_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'quaternion_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'quaternion_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'quaternion_w) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear_acceleration_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear_acceleration_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear_acceleration_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_velocity_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_velocity_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_velocity_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImuSmall>)))
  "Returns string type for a message object of type '<ImuSmall>"
  "samana_msgs/ImuSmall")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImuSmall)))
  "Returns string type for a message object of type 'ImuSmall"
  "samana_msgs/ImuSmall")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImuSmall>)))
  "Returns md5sum for a message object of type '<ImuSmall>"
  "c052128fb9568718800fa0ba7071e271")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImuSmall)))
  "Returns md5sum for a message object of type 'ImuSmall"
  "c052128fb9568718800fa0ba7071e271")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImuSmall>)))
  "Returns full string definition for message of type '<ImuSmall>"
  (cl:format cl:nil "std_msgs/Header header~%float32 quaternion_x~%float32 quaternion_y~%float32 quaternion_z~%float32 quaternion_w~%float32 linear_acceleration_x~%float32 linear_acceleration_y~%float32 linear_acceleration_z~%float32 angular_velocity_x~%float32 angular_velocity_y~%float32 angular_velocity_z~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImuSmall)))
  "Returns full string definition for message of type 'ImuSmall"
  (cl:format cl:nil "std_msgs/Header header~%float32 quaternion_x~%float32 quaternion_y~%float32 quaternion_z~%float32 quaternion_w~%float32 linear_acceleration_x~%float32 linear_acceleration_y~%float32 linear_acceleration_z~%float32 angular_velocity_x~%float32 angular_velocity_y~%float32 angular_velocity_z~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImuSmall>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImuSmall>))
  "Converts a ROS message object to a list"
  (cl:list 'ImuSmall
    (cl:cons ':header (header msg))
    (cl:cons ':quaternion_x (quaternion_x msg))
    (cl:cons ':quaternion_y (quaternion_y msg))
    (cl:cons ':quaternion_z (quaternion_z msg))
    (cl:cons ':quaternion_w (quaternion_w msg))
    (cl:cons ':linear_acceleration_x (linear_acceleration_x msg))
    (cl:cons ':linear_acceleration_y (linear_acceleration_y msg))
    (cl:cons ':linear_acceleration_z (linear_acceleration_z msg))
    (cl:cons ':angular_velocity_x (angular_velocity_x msg))
    (cl:cons ':angular_velocity_y (angular_velocity_y msg))
    (cl:cons ':angular_velocity_z (angular_velocity_z msg))
))
