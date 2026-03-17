; Auto-generated. Do not edit!


(cl:in-package limo_deeplearning-msg)


;//! \htmlinclude img.msg.html

(cl:defclass <img> (roslisp-msg-protocol:ros-message)
  ((img
    :reader img
    :initarg :img
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass img (<img>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <img>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'img)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name limo_deeplearning-msg:<img> is deprecated: use limo_deeplearning-msg:img instead.")))

(cl:ensure-generic-function 'img-val :lambda-list '(m))
(cl:defmethod img-val ((m <img>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_deeplearning-msg:img-val is deprecated.  Use limo_deeplearning-msg:img instead.")
  (img m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <img>) ostream)
  "Serializes a message object of type '<img>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'img))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'img))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <img>) istream)
  "Deserializes a message object of type '<img>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'img) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'img)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<img>)))
  "Returns string type for a message object of type '<img>"
  "limo_deeplearning/img")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'img)))
  "Returns string type for a message object of type 'img"
  "limo_deeplearning/img")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<img>)))
  "Returns md5sum for a message object of type '<img>"
  "06c52dd5e482fac9219b8508a9f9cc99")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'img)))
  "Returns md5sum for a message object of type 'img"
  "06c52dd5e482fac9219b8508a9f9cc99")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<img>)))
  "Returns full string definition for message of type '<img>"
  (cl:format cl:nil "int16[] img~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'img)))
  "Returns full string definition for message of type 'img"
  (cl:format cl:nil "int16[] img~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <img>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'img) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <img>))
  "Converts a ROS message object to a list"
  (cl:list 'img
    (cl:cons ':img (img msg))
))
