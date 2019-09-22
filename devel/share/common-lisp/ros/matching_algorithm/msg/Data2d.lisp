; Auto-generated. Do not edit!


(cl:in-package matching_algorithm-msg)


;//! \htmlinclude Data2d.msg.html

(cl:defclass <Data2d> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector matching_algorithm-msg:Data1d)
   :initform (cl:make-array 0 :element-type 'matching_algorithm-msg:Data1d :initial-element (cl:make-instance 'matching_algorithm-msg:Data1d))))
)

(cl:defclass Data2d (<Data2d>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Data2d>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Data2d)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name matching_algorithm-msg:<Data2d> is deprecated: use matching_algorithm-msg:Data2d instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Data2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader matching_algorithm-msg:data-val is deprecated.  Use matching_algorithm-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Data2d>) ostream)
  "Serializes a message object of type '<Data2d>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Data2d>) istream)
  "Deserializes a message object of type '<Data2d>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'matching_algorithm-msg:Data1d))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Data2d>)))
  "Returns string type for a message object of type '<Data2d>"
  "matching_algorithm/Data2d")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Data2d)))
  "Returns string type for a message object of type 'Data2d"
  "matching_algorithm/Data2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Data2d>)))
  "Returns md5sum for a message object of type '<Data2d>"
  "72cb3ba2313f1afdb0932a688ee31bc1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Data2d)))
  "Returns md5sum for a message object of type 'Data2d"
  "72cb3ba2313f1afdb0932a688ee31bc1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Data2d>)))
  "Returns full string definition for message of type '<Data2d>"
  (cl:format cl:nil "Data1d[] data~%~%~%================================================================================~%MSG: matching_algorithm/Data1d~%float64[] data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Data2d)))
  "Returns full string definition for message of type 'Data2d"
  (cl:format cl:nil "Data1d[] data~%~%~%================================================================================~%MSG: matching_algorithm/Data1d~%float64[] data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Data2d>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Data2d>))
  "Converts a ROS message object to a list"
  (cl:list 'Data2d
    (cl:cons ':data (data msg))
))
