; Auto-generated. Do not edit!


(cl:in-package qt_nuitrack_app-msg)


;//! \htmlinclude Hands.msg.html

(cl:defclass <Hands> (roslisp-msg-protocol:ros-message)
  ((hands
    :reader hands
    :initarg :hands
    :type (cl:vector qt_nuitrack_app-msg:HandInfo)
   :initform (cl:make-array 0 :element-type 'qt_nuitrack_app-msg:HandInfo :initial-element (cl:make-instance 'qt_nuitrack_app-msg:HandInfo))))
)

(cl:defclass Hands (<Hands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_nuitrack_app-msg:<Hands> is deprecated: use qt_nuitrack_app-msg:Hands instead.")))

(cl:ensure-generic-function 'hands-val :lambda-list '(m))
(cl:defmethod hands-val ((m <Hands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:hands-val is deprecated.  Use qt_nuitrack_app-msg:hands instead.")
  (hands m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hands>) ostream)
  "Serializes a message object of type '<Hands>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'hands))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'hands))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hands>) istream)
  "Deserializes a message object of type '<Hands>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'hands) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'hands)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'qt_nuitrack_app-msg:HandInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hands>)))
  "Returns string type for a message object of type '<Hands>"
  "qt_nuitrack_app/Hands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hands)))
  "Returns string type for a message object of type 'Hands"
  "qt_nuitrack_app/Hands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hands>)))
  "Returns md5sum for a message object of type '<Hands>"
  "633d76b336567f0906335e6cf0195299")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hands)))
  "Returns md5sum for a message object of type 'Hands"
  "633d76b336567f0906335e6cf0195299")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hands>)))
  "Returns full string definition for message of type '<Hands>"
  (cl:format cl:nil "#std_msgs/Header header~%HandInfo[] hands~%~%================================================================================~%MSG: qt_nuitrack_app/HandInfo~%#std_msgs/Header header~%int32 id~%#The normalized projective (x, y) coordinate of the right hand~%float32[] right_projection~%#The (x,y,z) coordinate of the right hand in the world system.~%float32[] right_real~%bool right_click~%int32 right_pressure~%#The normalized projective (x, y) coordinate of the left hand~%float32[] left_projection~%#The (x,y,z) coordinate of the left hand in the world system.~%float32[] left_real~%bool left_click~%int32 left_pressure~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hands)))
  "Returns full string definition for message of type 'Hands"
  (cl:format cl:nil "#std_msgs/Header header~%HandInfo[] hands~%~%================================================================================~%MSG: qt_nuitrack_app/HandInfo~%#std_msgs/Header header~%int32 id~%#The normalized projective (x, y) coordinate of the right hand~%float32[] right_projection~%#The (x,y,z) coordinate of the right hand in the world system.~%float32[] right_real~%bool right_click~%int32 right_pressure~%#The normalized projective (x, y) coordinate of the left hand~%float32[] left_projection~%#The (x,y,z) coordinate of the left hand in the world system.~%float32[] left_real~%bool left_click~%int32 left_pressure~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hands>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'hands) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hands>))
  "Converts a ROS message object to a list"
  (cl:list 'Hands
    (cl:cons ':hands (hands msg))
))
