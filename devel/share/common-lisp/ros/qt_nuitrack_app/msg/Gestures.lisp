; Auto-generated. Do not edit!


(cl:in-package qt_nuitrack_app-msg)


;//! \htmlinclude Gestures.msg.html

(cl:defclass <Gestures> (roslisp-msg-protocol:ros-message)
  ((gestures
    :reader gestures
    :initarg :gestures
    :type (cl:vector qt_nuitrack_app-msg:GestureInfo)
   :initform (cl:make-array 0 :element-type 'qt_nuitrack_app-msg:GestureInfo :initial-element (cl:make-instance 'qt_nuitrack_app-msg:GestureInfo))))
)

(cl:defclass Gestures (<Gestures>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Gestures>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Gestures)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_nuitrack_app-msg:<Gestures> is deprecated: use qt_nuitrack_app-msg:Gestures instead.")))

(cl:ensure-generic-function 'gestures-val :lambda-list '(m))
(cl:defmethod gestures-val ((m <Gestures>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:gestures-val is deprecated.  Use qt_nuitrack_app-msg:gestures instead.")
  (gestures m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Gestures>) ostream)
  "Serializes a message object of type '<Gestures>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gestures))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'gestures))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Gestures>) istream)
  "Deserializes a message object of type '<Gestures>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gestures) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gestures)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'qt_nuitrack_app-msg:GestureInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Gestures>)))
  "Returns string type for a message object of type '<Gestures>"
  "qt_nuitrack_app/Gestures")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Gestures)))
  "Returns string type for a message object of type 'Gestures"
  "qt_nuitrack_app/Gestures")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Gestures>)))
  "Returns md5sum for a message object of type '<Gestures>"
  "cb33e599df6d94bfd33170149d8d8cba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Gestures)))
  "Returns md5sum for a message object of type 'Gestures"
  "cb33e599df6d94bfd33170149d8d8cba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Gestures>)))
  "Returns full string definition for message of type '<Gestures>"
  (cl:format cl:nil "#std_msgs/Header header~%GestureInfo[] gestures~%~%================================================================================~%MSG: qt_nuitrack_app/GestureInfo~%#std_msgs/Header header~%int32 id~%string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Gestures)))
  "Returns full string definition for message of type 'Gestures"
  (cl:format cl:nil "#std_msgs/Header header~%GestureInfo[] gestures~%~%================================================================================~%MSG: qt_nuitrack_app/GestureInfo~%#std_msgs/Header header~%int32 id~%string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Gestures>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gestures) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Gestures>))
  "Converts a ROS message object to a list"
  (cl:list 'Gestures
    (cl:cons ':gestures (gestures msg))
))
