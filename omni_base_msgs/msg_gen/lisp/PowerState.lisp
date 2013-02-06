; Auto-generated. Do not edit!


(cl:in-package omni_base_msgs-msg)


;//! \htmlinclude PowerState.msg.html

(cl:defclass <PowerState> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (enabled
    :reader enabled
    :initarg :enabled
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PowerState (<PowerState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PowerState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PowerState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name omni_base_msgs-msg:<PowerState> is deprecated: use omni_base_msgs-msg:PowerState instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <PowerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_base_msgs-msg:name-val is deprecated.  Use omni_base_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <PowerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_base_msgs-msg:enabled-val is deprecated.  Use omni_base_msgs-msg:enabled instead.")
  (enabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PowerState>) ostream)
  "Serializes a message object of type '<PowerState>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PowerState>) istream)
  "Deserializes a message object of type '<PowerState>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'enabled) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PowerState>)))
  "Returns string type for a message object of type '<PowerState>"
  "omni_base_msgs/PowerState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PowerState)))
  "Returns string type for a message object of type 'PowerState"
  "omni_base_msgs/PowerState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PowerState>)))
  "Returns md5sum for a message object of type '<PowerState>"
  "0a25a4ba6b569752be722a73fae9c6c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PowerState)))
  "Returns md5sum for a message object of type 'PowerState"
  "0a25a4ba6b569752be722a73fae9c6c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PowerState>)))
  "Returns full string definition for message of type '<PowerState>"
  (cl:format cl:nil "string name  # name of the relay / actuator ~%bool enabled # whether the relay is closed / the actuator has power~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PowerState)))
  "Returns full string definition for message of type 'PowerState"
  (cl:format cl:nil "string name  # name of the relay / actuator ~%bool enabled # whether the relay is closed / the actuator has power~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PowerState>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PowerState>))
  "Converts a ROS message object to a list"
  (cl:list 'PowerState
    (cl:cons ':name (name msg))
    (cl:cons ':enabled (enabled msg))
))
