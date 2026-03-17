; Auto-generated. Do not edit!


(cl:in-package limo_deeplearning-msg)


;//! \htmlinclude traffic.msg.html

(cl:defclass <traffic> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (number
    :reader number
    :initarg :number
    :type cl:fixnum
    :initform 0))
)

(cl:defclass traffic (<traffic>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <traffic>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'traffic)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name limo_deeplearning-msg:<traffic> is deprecated: use limo_deeplearning-msg:traffic instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <traffic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_deeplearning-msg:name-val is deprecated.  Use limo_deeplearning-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'number-val :lambda-list '(m))
(cl:defmethod number-val ((m <traffic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_deeplearning-msg:number-val is deprecated.  Use limo_deeplearning-msg:number instead.")
  (number m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <traffic>) ostream)
  "Serializes a message object of type '<traffic>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'number)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <traffic>) istream)
  "Deserializes a message object of type '<traffic>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'number)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<traffic>)))
  "Returns string type for a message object of type '<traffic>"
  "limo_deeplearning/traffic")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'traffic)))
  "Returns string type for a message object of type 'traffic"
  "limo_deeplearning/traffic")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<traffic>)))
  "Returns md5sum for a message object of type '<traffic>"
  "29bb8ce5bfb548d08c8f5eabb122ba14")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'traffic)))
  "Returns md5sum for a message object of type 'traffic"
  "29bb8ce5bfb548d08c8f5eabb122ba14")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<traffic>)))
  "Returns full string definition for message of type '<traffic>"
  (cl:format cl:nil "string name~%uint16 number~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'traffic)))
  "Returns full string definition for message of type 'traffic"
  (cl:format cl:nil "string name~%uint16 number~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <traffic>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <traffic>))
  "Converts a ROS message object to a list"
  (cl:list 'traffic
    (cl:cons ':name (name msg))
    (cl:cons ':number (number msg))
))
