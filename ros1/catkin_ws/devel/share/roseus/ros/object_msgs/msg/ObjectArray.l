;; Auto-generated. Do not edit!


(when (boundp 'object_msgs::ObjectArray)
  (if (not (find-package "OBJECT_MSGS"))
    (make-package "OBJECT_MSGS"))
  (shadow 'ObjectArray (find-package "OBJECT_MSGS")))
(unless (find-package "OBJECT_MSGS::OBJECTARRAY")
  (make-package "OBJECT_MSGS::OBJECTARRAY"))

(in-package "ROS")
;;//! \htmlinclude ObjectArray.msg.html


(defclass object_msgs::ObjectArray
  :super ros::object
  :slots (_Objects ))

(defmethod object_msgs::ObjectArray
  (:init
   (&key
    ((:Objects __Objects) ())
    )
   (send-super :init)
   (setq _Objects __Objects)
   self)
  (:Objects
   (&rest __Objects)
   (if (keywordp (car __Objects))
       (send* _Objects __Objects)
     (progn
       (if __Objects (setq _Objects (car __Objects)))
       _Objects)))
  (:serialization-length
   ()
   (+
    ;; object_msgs/Object[] _Objects
    (apply #'+ (send-all _Objects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; object_msgs/Object[] _Objects
     (write-long (length _Objects) s)
     (dolist (elem _Objects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; object_msgs/Object[] _Objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _Objects (let (r) (dotimes (i n) (push (instance object_msgs::Object :init) r)) r))
     (dolist (elem- _Objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get object_msgs::ObjectArray :md5sum-) "a82955a978c818e847a2c852e8536db8")
(setf (get object_msgs::ObjectArray :datatype-) "object_msgs/ObjectArray")
(setf (get object_msgs::ObjectArray :definition-)
      "Object[] Objects
================================================================================
MSG: object_msgs/Object
string class_name
uint32[] xmin_ymin_xmax_ymax
")



(provide :object_msgs/ObjectArray "a82955a978c818e847a2c852e8536db8")


