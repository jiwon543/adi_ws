; Auto-generated. Do not edit!


(cl:in-package limo_deeplearning-msg)


;//! \htmlinclude light.msg.html

(cl:defclass <light> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:string
    :initform ""))
)

(cl:defclass light (<light>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <light>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'light)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name limo_deeplearning-msg:<light> is deprecated: use limo_deeplearning-msg:light instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <light>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_deeplearning-msg:mode-val is deprecated.  Use limo_deeplearning-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <light>) ostream)
  "Serializes a message object of type '<light>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mode))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <light>) istream)
  "Deserializes a message object of type '<light>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<light>)))
  "Returns string type for a message object of type '<light>"
  "limo_deeplearning/light")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'light)))
  "Returns string type for a message object of type 'light"
  "limo_deeplearning/light")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<light>)))
  "Returns md5sum for a message object of type '<light>"
  "e84dc3ad5dc323bb64f0aca01c2d1eef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'light)))
  "Returns md5sum for a message object of type 'light"
  "e84dc3ad5dc323bb64f0aca01c2d1eef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<light>)))
  "Returns full string definition for message of type '<light>"
  (cl:format cl:nil "string mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'light)))
  "Returns full string definition for message of type 'light"
  (cl:format cl:nil "string mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <light>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'mode))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <light>))
  "Converts a ROS message object to a list"
  (cl:list 'light
    (cl:cons ':mode (mode msg))
))
