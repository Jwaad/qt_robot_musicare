; Auto-generated. Do not edit!


(cl:in-package qt_nuitrack_app-msg)


;//! \htmlinclude Skeletons.msg.html

(cl:defclass <Skeletons> (roslisp-msg-protocol:ros-message)
  ((skeletons
    :reader skeletons
    :initarg :skeletons
    :type (cl:vector qt_nuitrack_app-msg:SkeletonInfo)
   :initform (cl:make-array 0 :element-type 'qt_nuitrack_app-msg:SkeletonInfo :initial-element (cl:make-instance 'qt_nuitrack_app-msg:SkeletonInfo))))
)

(cl:defclass Skeletons (<Skeletons>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Skeletons>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Skeletons)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_nuitrack_app-msg:<Skeletons> is deprecated: use qt_nuitrack_app-msg:Skeletons instead.")))

(cl:ensure-generic-function 'skeletons-val :lambda-list '(m))
(cl:defmethod skeletons-val ((m <Skeletons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:skeletons-val is deprecated.  Use qt_nuitrack_app-msg:skeletons instead.")
  (skeletons m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Skeletons>) ostream)
  "Serializes a message object of type '<Skeletons>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'skeletons))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'skeletons))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Skeletons>) istream)
  "Deserializes a message object of type '<Skeletons>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'skeletons) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'skeletons)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'qt_nuitrack_app-msg:SkeletonInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Skeletons>)))
  "Returns string type for a message object of type '<Skeletons>"
  "qt_nuitrack_app/Skeletons")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Skeletons)))
  "Returns string type for a message object of type 'Skeletons"
  "qt_nuitrack_app/Skeletons")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Skeletons>)))
  "Returns md5sum for a message object of type '<Skeletons>"
  "c9e6a0abe2b44381a0de593ee0c2c308")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Skeletons)))
  "Returns md5sum for a message object of type 'Skeletons"
  "c9e6a0abe2b44381a0de593ee0c2c308")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Skeletons>)))
  "Returns full string definition for message of type '<Skeletons>"
  (cl:format cl:nil "#std_msgs/Header header~%SkeletonInfo[] skeletons~%~%================================================================================~%MSG: qt_nuitrack_app/SkeletonInfo~%#std_msgs/Header header~%int32 id~%JointInfo[] joints ~%~%================================================================================~%MSG: qt_nuitrack_app/JointInfo~%#std_msgs/Header header~%uint8 type~%float32 confidence~%float32[] real~%float32[] projection~%float32[] orientation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Skeletons)))
  "Returns full string definition for message of type 'Skeletons"
  (cl:format cl:nil "#std_msgs/Header header~%SkeletonInfo[] skeletons~%~%================================================================================~%MSG: qt_nuitrack_app/SkeletonInfo~%#std_msgs/Header header~%int32 id~%JointInfo[] joints ~%~%================================================================================~%MSG: qt_nuitrack_app/JointInfo~%#std_msgs/Header header~%uint8 type~%float32 confidence~%float32[] real~%float32[] projection~%float32[] orientation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Skeletons>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'skeletons) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Skeletons>))
  "Converts a ROS message object to a list"
  (cl:list 'Skeletons
    (cl:cons ':skeletons (skeletons msg))
))
