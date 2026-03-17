;; Auto-generated. Do not edit!


(when (boundp 'object_msgs::Object)
  (if (not (find-package "OBJECT_MSGS"))
    (make-package "OBJECT_MSGS"))
  (shadow 'Object (find-package "OBJECT_MSGS")))
(unless (find-package "OBJECT_MSGS::OBJECT")
  (make-package "OBJECT_MSGS::OBJECT"))

(in-package "ROS")
;;//! \htmlinclude Object.msg.html


(defclass object_msgs::Object
  :super ros::object
  :slots (_class_name _xmin_ymin_xmax_ymax ))

(defmethod object_msgs::Object
  (:init
   (&key
    ((:class_name __class_name) "")
    ((:xmin_ymin_xmax_ymax __xmin_ymin_xmax_ymax) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _class_name (string __class_name))
   (setq _xmin_ymin_xmax_ymax __xmin_ymin_xmax_ymax)
   self)
  (:class_name
   (&optional __class_name)
   (if __class_name (setq _class_name __class_name)) _class_name)
  (:xmin_ymin_xmax_ymax
   (&optional __xmin_ymin_xmax_ymax)
   (if __xmin_ymin_xmax_ymax (setq _xmin_ymin_xmax_ymax __xmin_ymin_xmax_ymax)) _xmin_ymin_xmax_ymax)
  (:serialization-length
   ()
   (+
    ;; string _class_name
    4 (length _class_name)
    ;; uint32[] _xmin_ymin_xmax_ymax
    (* 4    (length _xmin_ymin_xmax_ymax)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _class_name
       (write-long (length _class_name) s) (princ _class_name s)
     ;; uint32[] _xmin_ymin_xmax_ymax
     (write-long (length _xmin_ymin_xmax_ymax) s)
     (dotimes (i (length _xmin_ymin_xmax_ymax))
       (write-long (elt _xmin_ymin_xmax_ymax i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _class_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _class_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32[] _xmin_ymin_xmax_ymax
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _xmin_ymin_xmax_ymax (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _xmin_ymin_xmax_ymax i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get object_msgs::Object :md5sum-) "f9beacd4cdb37d3951c7c33ecebfabd1")
(setf (get object_msgs::Object :datatype-) "object_msgs/Object")
(setf (get object_msgs::Object :definition-)
      "string class_name
uint32[] xmin_ymin_xmax_ymax
")



(provide :object_msgs/Object "f9beacd4cdb37d3951c7c33ecebfabd1")


