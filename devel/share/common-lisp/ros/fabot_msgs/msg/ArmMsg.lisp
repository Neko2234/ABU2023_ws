; Auto-generated. Do not edit!


(cl:in-package fabot_msgs-msg)


;//! \htmlinclude ArmMsg.msg.html

(cl:defclass <ArmMsg> (roslisp-msg-protocol:ros-message)
  ((hand
    :reader hand
    :initarg :hand
    :type cl:fixnum
    :initform 0)
   (arm
    :reader arm
    :initarg :arm
    :type cl:fixnum
    :initform 0)
   (hand_duty
    :reader hand_duty
    :initarg :hand_duty
    :type cl:fixnum
    :initform 0)
   (arm_duty
    :reader arm_duty
    :initarg :arm_duty
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ArmMsg (<ArmMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fabot_msgs-msg:<ArmMsg> is deprecated: use fabot_msgs-msg:ArmMsg instead.")))

(cl:ensure-generic-function 'hand-val :lambda-list '(m))
(cl:defmethod hand-val ((m <ArmMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fabot_msgs-msg:hand-val is deprecated.  Use fabot_msgs-msg:hand instead.")
  (hand m))

(cl:ensure-generic-function 'arm-val :lambda-list '(m))
(cl:defmethod arm-val ((m <ArmMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fabot_msgs-msg:arm-val is deprecated.  Use fabot_msgs-msg:arm instead.")
  (arm m))

(cl:ensure-generic-function 'hand_duty-val :lambda-list '(m))
(cl:defmethod hand_duty-val ((m <ArmMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fabot_msgs-msg:hand_duty-val is deprecated.  Use fabot_msgs-msg:hand_duty instead.")
  (hand_duty m))

(cl:ensure-generic-function 'arm_duty-val :lambda-list '(m))
(cl:defmethod arm_duty-val ((m <ArmMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fabot_msgs-msg:arm_duty-val is deprecated.  Use fabot_msgs-msg:arm_duty instead.")
  (arm_duty m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmMsg>) ostream)
  "Serializes a message object of type '<ArmMsg>"
  (cl:let* ((signed (cl:slot-value msg 'hand)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'arm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'hand_duty)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'arm_duty)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmMsg>) istream)
  "Deserializes a message object of type '<ArmMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hand) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arm) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hand_duty) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arm_duty) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmMsg>)))
  "Returns string type for a message object of type '<ArmMsg>"
  "fabot_msgs/ArmMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmMsg)))
  "Returns string type for a message object of type 'ArmMsg"
  "fabot_msgs/ArmMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmMsg>)))
  "Returns md5sum for a message object of type '<ArmMsg>"
  "7e5814a5ce25cfec12c99674e013cee3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmMsg)))
  "Returns md5sum for a message object of type 'ArmMsg"
  "7e5814a5ce25cfec12c99674e013cee3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmMsg>)))
  "Returns full string definition for message of type '<ArmMsg>"
  (cl:format cl:nil "int16 hand~%int16 arm~%int16 hand_duty~%int16 arm_duty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmMsg)))
  "Returns full string definition for message of type 'ArmMsg"
  (cl:format cl:nil "int16 hand~%int16 arm~%int16 hand_duty~%int16 arm_duty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmMsg>))
  (cl:+ 0
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmMsg
    (cl:cons ':hand (hand msg))
    (cl:cons ':arm (arm msg))
    (cl:cons ':hand_duty (hand_duty msg))
    (cl:cons ':arm_duty (arm_duty msg))
))
