;; Auto-generated. Do not edit!


(when (boundp 'matching_algorithm::Data1d)
  (if (not (find-package "MATCHING_ALGORITHM"))
    (make-package "MATCHING_ALGORITHM"))
  (shadow 'Data1d (find-package "MATCHING_ALGORITHM")))
(unless (find-package "MATCHING_ALGORITHM::DATA1D")
  (make-package "MATCHING_ALGORITHM::DATA1D"))

(in-package "ROS")
;;//! \htmlinclude Data1d.msg.html


(defclass matching_algorithm::Data1d
  :super ros::object
  :slots (_data ))

(defmethod matching_algorithm::Data1d
  (:init
   (&key
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float64[] _data
    (* 8    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get matching_algorithm::Data1d :md5sum-) "788898178a3da2c3718461eecda8f714")
(setf (get matching_algorithm::Data1d :datatype-) "matching_algorithm/Data1d")
(setf (get matching_algorithm::Data1d :definition-)
      "float64[] data


")



(provide :matching_algorithm/Data1d "788898178a3da2c3718461eecda8f714")


