; Auto-generated. Do not edit!


(cl:in-package adbot_msgs-msg)


;//! \htmlinclude SprMsg.msg.html

(cl:defclass <SprMsg> (roslisp-msg-protocol:ros-message)
  ((isOn
    :reader isOn
    :initarg :isOn
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SprMsg (<SprMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SprMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SprMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name adbot_msgs-msg:<SprMsg> is deprecated: use adbot_msgs-msg:SprMsg instead.")))

(cl:ensure-generic-function 'isOn-val :lambda-list '(m))
(cl:defmethod isOn-val ((m <SprMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adbot_msgs-msg:isOn-val is deprecated.  Use adbot_msgs-msg:isOn instead.")
  (isOn m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SprMsg>) ostream)
  "Serializes a message object of type '<SprMsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isOn) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SprMsg>) istream)
  "Deserializes a message object of type '<SprMsg>"
    (cl:setf (cl:slot-value msg 'isOn) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SprMsg>)))
  "Returns string type for a message object of type '<SprMsg>"
  "adbot_msgs/SprMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SprMsg)))
  "Returns string type for a message object of type 'SprMsg"
  "adbot_msgs/SprMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SprMsg>)))
  "Returns md5sum for a message object of type '<SprMsg>"
  "7815df4e3625e249c3731540caef0458")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SprMsg)))
  "Returns md5sum for a message object of type 'SprMsg"
  "7815df4e3625e249c3731540caef0458")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SprMsg>)))
  "Returns full string definition for message of type '<SprMsg>"
  (cl:format cl:nil "bool isOn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SprMsg)))
  "Returns full string definition for message of type 'SprMsg"
  (cl:format cl:nil "bool isOn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SprMsg>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SprMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'SprMsg
    (cl:cons ':isOn (isOn msg))
))
