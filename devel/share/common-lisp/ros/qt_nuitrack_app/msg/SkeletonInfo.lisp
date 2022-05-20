; Auto-generated. Do not edit!


(cl:in-package qt_nuitrack_app-msg)


;//! \htmlinclude SkeletonInfo.msg.html

(cl:defclass <SkeletonInfo> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (joints
    :reader joints
    :initarg :joints
    :type (cl:vector qt_nuitrack_app-msg:JointInfo)
   :initform (cl:make-array 0 :element-type 'qt_nuitrack_app-msg:JointInfo :initial-element (cl:make-instance 'qt_nuitrack_app-msg:JointInfo))))
)

(cl:defclass SkeletonInfo (<SkeletonInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SkeletonInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SkeletonInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_nuitrack_app-msg:<SkeletonInfo> is deprecated: use qt_nuitrack_app-msg:SkeletonInfo instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SkeletonInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:id-val is deprecated.  Use qt_nuitrack_app-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'joints-val :lambda-list '(m))
(cl:defmethod joints-val ((m <SkeletonInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:joints-val is deprecated.  Use qt_nuitrack_app-msg:joints instead.")
  (joints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SkeletonInfo>) ostream)
  "Serializes a message object of type '<SkeletonInfo>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'joints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SkeletonInfo>) istream)
  "Deserializes a message object of type '<SkeletonInfo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'qt_nuitrack_app-msg:JointInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SkeletonInfo>)))
  "Returns string type for a message object of type '<SkeletonInfo>"
  "qt_nuitrack_app/SkeletonInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SkeletonInfo)))
  "Returns string type for a message object of type 'SkeletonInfo"
  "qt_nuitrack_app/SkeletonInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SkeletonInfo>)))
  "Returns md5sum for a message object of type '<SkeletonInfo>"
  "ac75876d5ead18b4ad4603a6b5b71a97")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SkeletonInfo)))
  "Returns md5sum for a message object of type 'SkeletonInfo"
  "ac75876d5ead18b4ad4603a6b5b71a97")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SkeletonInfo>)))
  "Returns full string definition for message of type '<SkeletonInfo>"
  (cl:format cl:nil "#std_msgs/Header header~%int32 id~%JointInfo[] joints ~%~%================================================================================~%MSG: qt_nuitrack_app/JointInfo~%#std_msgs/Header header~%uint8 type~%float32 confidence~%float32[] real~%float32[] projection~%float32[] orientation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SkeletonInfo)))
  "Returns full string definition for message of type 'SkeletonInfo"
  (cl:format cl:nil "#std_msgs/Header header~%int32 id~%JointInfo[] joints ~%~%================================================================================~%MSG: qt_nuitrack_app/JointInfo~%#std_msgs/Header header~%uint8 type~%float32 confidence~%float32[] real~%float32[] projection~%float32[] orientation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SkeletonInfo>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SkeletonInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'SkeletonInfo
    (cl:cons ':id (id msg))
    (cl:cons ':joints (joints msg))
))
