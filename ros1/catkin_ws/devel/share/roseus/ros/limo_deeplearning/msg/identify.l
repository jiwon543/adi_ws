;; Auto-generated. Do not edit!


(when (boundp 'limo_deeplearning::identify)
  (if (not (find-package "LIMO_DEEPLEARNING"))
    (make-package "LIMO_DEEPLEARNING"))
  (shadow 'identify (find-package "LIMO_DEEPLEARNING")))
(unless (find-package "LIMO_DEEPLEARNING::IDENTIFY")
  (make-package "LIMO_DEEPLEARNING::IDENTIFY"))

(in-package "ROS")
;;//! \htmlinclude identify.msg.html


(defclass limo_deeplearning::identify
  :super ros::object
  :slots (_results _classes _area _position _acc _image_number ))

(defmethod limo_deeplearning::identify
  (:init
   (&key
    ((:results __results) "")
    ((:classes __classes) 0)
    ((:area __area) 0)
    ((:position __position) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:acc __acc) 0.0)
    ((:image_number __image_number) 0)
    )
   (send-super :init)
   (setq _results (string __results))
   (setq _classes (round __classes))
   (setq _area (round __area))
   (setq _position __position)
   (setq _acc (float __acc))
   (setq _image_number (round __image_number))
   self)
  (:results
   (&optional __results)
   (if __results (setq _results __results)) _results)
  (:classes
   (&optional __classes)
   (if __classes (setq _classes __classes)) _classes)
  (:area
   (&optional __area)
   (if __area (setq _area __area)) _area)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:image_number
   (&optional __image_number)
   (if __image_number (setq _image_number __image_number)) _image_number)
  (:serialization-length
   ()
   (+
    ;; string _results
    4 (length _results)
    ;; int8 _classes
    1
    ;; int32 _area
    4
    ;; float32[4] _position
    (* 4    4)
    ;; float32 _acc
    4
    ;; int32 _image_number
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _results
       (write-long (length _results) s) (princ _results s)
     ;; int8 _classes
       (write-byte _classes s)
     ;; int32 _area
       (write-long _area s)
     ;; float32[4] _position
     (dotimes (i 4)
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _acc
       (sys::poke _acc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _image_number
       (write-long _image_number s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _results
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _results (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _classes
     (setq _classes (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _classes 127) (setq _classes (- _classes 256)))
   ;; int32 _area
     (setq _area (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[4] _position
   (dotimes (i (length _position))
     (setf (elt _position i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32 _acc
     (setq _acc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _image_number
     (setq _image_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get limo_deeplearning::identify :md5sum-) "f6053d8d260d6714b1da39ebe6fbf86b")
(setf (get limo_deeplearning::identify :datatype-) "limo_deeplearning/identify")
(setf (get limo_deeplearning::identify :definition-)
      "string results
int8 classes
int32 area
float32[4] position
float32 acc
int32 image_number

")



(provide :limo_deeplearning/identify "f6053d8d260d6714b1da39ebe6fbf86b")


