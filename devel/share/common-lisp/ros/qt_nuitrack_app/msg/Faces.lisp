; Auto-generated. Do not edit!


(cl:in-package qt_nuitrack_app-msg)


;//! \htmlinclude Faces.msg.html

(cl:defclass <Faces> (roslisp-msg-protocol:ros-message)
  ((faces
    :reader faces
    :initarg :faces
    :type (cl:vector qt_nuitrack_app-msg:FaceInfo)
   :initform (cl:make-array 0 :element-type 'qt_nuitrack_app-msg:FaceInfo :initial-element (cl:make-instance 'qt_nuitrack_app-msg:FaceInfo))))
)

(cl:defclass Faces (<Faces>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Faces>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Faces)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_nuitrack_app-msg:<Faces> is deprecated: use qt_nuitrack_app-msg:Faces instead.")))

(cl:ensure-generic-function 'faces-val :lambda-list '(m))
(cl:defmethod faces-val ((m <Faces>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:faces-val is deprecated.  Use qt_nuitrack_app-msg:faces instead.")
  (faces m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Faces>) ostream)
  "Serializes a message object of type '<Faces>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'faces))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'faces))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Faces>) istream)
  "Deserializes a message object of type '<Faces>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'faces) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'faces)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'qt_nuitrack_app-msg:FaceInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Faces>)))
  "Returns string type for a message object of type '<Faces>"
  "qt_nuitrack_app/Faces")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Faces)))
  "Returns string type for a message object of type 'Faces"
  "qt_nuitrack_app/Faces")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Faces>)))
  "Returns md5sum for a message object of type '<Faces>"
  "20dfb5db1e90db4dafd45bc663893505")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Faces)))
  "Returns md5sum for a message object of type 'Faces"
  "20dfb5db1e90db4dafd45bc663893505")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Faces>)))
  "Returns full string definition for message of type '<Faces>"
  (cl:format cl:nil "#std_msgs/Header header~%FaceInfo[] faces~%~%================================================================================~%MSG: qt_nuitrack_app/FaceInfo~%#std_msgs/Header header~%int32 id~%string gender~%int32 age_years~%string age_type~%float64 emotion_neutral~%float64 emotion_angry~%float64 emotion_happy~%float64 emotion_surprise~%#normalized screen coordinates of a face rectangle in the image (x, y, w, h)~%float64[] rectangle~%#normalized coordinates of the center of a person’s left eye (x,y)~%float64[] left_eye~%#normalized coordinates of the center of a person’s right eye (x,y)~%float64[] right_eye~%#face orientation angles in degrees (yaw, pitch, roll)~%float64[] angles~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Faces)))
  "Returns full string definition for message of type 'Faces"
  (cl:format cl:nil "#std_msgs/Header header~%FaceInfo[] faces~%~%================================================================================~%MSG: qt_nuitrack_app/FaceInfo~%#std_msgs/Header header~%int32 id~%string gender~%int32 age_years~%string age_type~%float64 emotion_neutral~%float64 emotion_angry~%float64 emotion_happy~%float64 emotion_surprise~%#normalized screen coordinates of a face rectangle in the image (x, y, w, h)~%float64[] rectangle~%#normalized coordinates of the center of a person’s left eye (x,y)~%float64[] left_eye~%#normalized coordinates of the center of a person’s right eye (x,y)~%float64[] right_eye~%#face orientation angles in degrees (yaw, pitch, roll)~%float64[] angles~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Faces>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'faces) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Faces>))
  "Converts a ROS message object to a list"
  (cl:list 'Faces
    (cl:cons ':faces (faces msg))
))
