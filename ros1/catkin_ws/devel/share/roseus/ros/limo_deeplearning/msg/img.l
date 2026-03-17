;; Auto-generated. Do not edit!


(when (boundp 'limo_deeplearning::img)
  (if (not (find-package "LIMO_DEEPLEARNING"))
    (make-package "LIMO_DEEPLEARNING"))
  (shadow 'img (find-package "LIMO_DEEPLEARNING")))
(unless (find-package "LIMO_DEEPLEARNING::IMG")
  (make-package "LIMO_DEEPLEARNING::IMG"))

(in-package "ROS")
;;//! \htmlinclude img.msg.html


(defclass limo_deeplearning::img
  :super ros::object
  :slots (_img ))

(defmethod limo_deeplearning::img
  (:init
   (&key
    ((:img __img) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _img __img)
   self)
  (:img
   (&optional __img)
   (if __img (setq _img __img)) _img)
  (:serialization-length
   ()
   (+
    ;; int16[] _img
    (* 2    (length _img)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16[] _img
     (write-long (length _img) s)
     (dotimes (i (length _img))
       (write-word (elt _img i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16[] _img
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _img (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _img i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get limo_deeplearning::img :md5sum-) "06c52dd5e482fac9219b8508a9f9cc99")
(setf (get limo_deeplearning::img :datatype-) "limo_deeplearning/img")
(setf (get limo_deeplearning::img :definition-)
      "int16[] img

")



(provide :limo_deeplearning/img "06c52dd5e482fac9219b8508a9f9cc99")


