;; Auto-generated. Do not edit!


(when (boundp 'matching_algorithm::Data2d)
  (if (not (find-package "MATCHING_ALGORITHM"))
    (make-package "MATCHING_ALGORITHM"))
  (shadow 'Data2d (find-package "MATCHING_ALGORITHM")))
(unless (find-package "MATCHING_ALGORITHM::DATA2D")
  (make-package "MATCHING_ALGORITHM::DATA2D"))

(in-package "ROS")
;;//! \htmlinclude Data2d.msg.html


(defclass matching_algorithm::Data2d
  :super ros::object
  :slots (_data ))

(defmethod matching_algorithm::Data2d
  (:init
   (&key
    ((:data __data) (let (r) (dotimes (i 0) (push (instance matching_algorithm::Data1d :init) r)) r))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; matching_algorithm/Data1d[] _data
    (apply #'+ (send-all _data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; matching_algorithm/Data1d[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; matching_algorithm/Data1d[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (let (r) (dotimes (i n) (push (instance matching_algorithm::Data1d :init) r)) r))
     (dolist (elem- _data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get matching_algorithm::Data2d :md5sum-) "72cb3ba2313f1afdb0932a688ee31bc1")
(setf (get matching_algorithm::Data2d :datatype-) "matching_algorithm/Data2d")
(setf (get matching_algorithm::Data2d :definition-)
      "Data1d[] data


================================================================================
MSG: matching_algorithm/Data1d
float64[] data


")



(provide :matching_algorithm/Data2d "72cb3ba2313f1afdb0932a688ee31bc1")


