; Auto-generated. Do not edit!


(cl:in-package limo_deeplearning-msg)


;//! \htmlinclude identify.msg.html

(cl:defclass <identify> (roslisp-msg-protocol:ros-message)
  ((results
    :reader results
    :initarg :results
    :type cl:string
    :initform "")
   (classes
    :reader classes
    :initarg :classes
    :type cl:fixnum
    :initform 0)
   (area
    :reader area
    :initarg :area
    :type cl:integer
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (acc
    :reader acc
    :initarg :acc
    :type cl:float
    :initform 0.0)
   (image_number
    :reader image_number
    :initarg :image_number
    :type cl:integer
    :initform 0))
)

(cl:defclass identify (<identify>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <identify>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'identify)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name limo_deeplearning-msg:<identify> is deprecated: use limo_deeplearning-msg:identify instead.")))

(cl:ensure-generic-function 'results-val :lambda-list '(m))
(cl:defmethod results-val ((m <identify>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_deeplearning-msg:results-val is deprecated.  Use limo_deeplearning-msg:results instead.")
  (results m))

(cl:ensure-generic-function 'classes-val :lambda-list '(m))
(cl:defmethod classes-val ((m <identify>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_deeplearning-msg:classes-val is deprecated.  Use limo_deeplearning-msg:classes instead.")
  (classes m))

(cl:ensure-generic-function 'area-val :lambda-list '(m))
(cl:defmethod area-val ((m <identify>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_deeplearning-msg:area-val is deprecated.  Use limo_deeplearning-msg:area instead.")
  (area m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <identify>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_deeplearning-msg:position-val is deprecated.  Use limo_deeplearning-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <identify>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_deeplearning-msg:acc-val is deprecated.  Use limo_deeplearning-msg:acc instead.")
  (acc m))

(cl:ensure-generic-function 'image_number-val :lambda-list '(m))
(cl:defmethod image_number-val ((m <identify>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_deeplearning-msg:image_number-val is deprecated.  Use limo_deeplearning-msg:image_number instead.")
  (image_number m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <identify>) ostream)
  "Serializes a message object of type '<identify>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'results))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'results))
  (cl:let* ((signed (cl:slot-value msg 'classes)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'area)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'position))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'acc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'image_number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <identify>) istream)
  "Deserializes a message object of type '<identify>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'results) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'results) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'classes) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'area) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'position) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'position)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'image_number) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<identify>)))
  "Returns string type for a message object of type '<identify>"
  "limo_deeplearning/identify")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'identify)))
  "Returns string type for a message object of type 'identify"
  "limo_deeplearning/identify")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<identify>)))
  "Returns md5sum for a message object of type '<identify>"
  "f6053d8d260d6714b1da39ebe6fbf86b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'identify)))
  "Returns md5sum for a message object of type 'identify"
  "f6053d8d260d6714b1da39ebe6fbf86b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<identify>)))
  "Returns full string definition for message of type '<identify>"
  (cl:format cl:nil "string results~%int8 classes~%int32 area~%float32[4] position~%float32 acc~%int32 image_number~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'identify)))
  "Returns full string definition for message of type 'identify"
  (cl:format cl:nil "string results~%int8 classes~%int32 area~%float32[4] position~%float32 acc~%int32 image_number~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <identify>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'results))
     1
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <identify>))
  "Converts a ROS message object to a list"
  (cl:list 'identify
    (cl:cons ':results (results msg))
    (cl:cons ':classes (classes msg))
    (cl:cons ':area (area msg))
    (cl:cons ':position (position msg))
    (cl:cons ':acc (acc msg))
    (cl:cons ':image_number (image_number msg))
))
