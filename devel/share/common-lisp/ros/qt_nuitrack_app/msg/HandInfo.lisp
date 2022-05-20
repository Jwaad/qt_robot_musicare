; Auto-generated. Do not edit!


(cl:in-package qt_nuitrack_app-msg)


;//! \htmlinclude HandInfo.msg.html

(cl:defclass <HandInfo> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (right_projection
    :reader right_projection
    :initarg :right_projection
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (right_real
    :reader right_real
    :initarg :right_real
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (right_click
    :reader right_click
    :initarg :right_click
    :type cl:boolean
    :initform cl:nil)
   (right_pressure
    :reader right_pressure
    :initarg :right_pressure
    :type cl:integer
    :initform 0)
   (left_projection
    :reader left_projection
    :initarg :left_projection
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (left_real
    :reader left_real
    :initarg :left_real
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (left_click
    :reader left_click
    :initarg :left_click
    :type cl:boolean
    :initform cl:nil)
   (left_pressure
    :reader left_pressure
    :initarg :left_pressure
    :type cl:integer
    :initform 0))
)

(cl:defclass HandInfo (<HandInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_nuitrack_app-msg:<HandInfo> is deprecated: use qt_nuitrack_app-msg:HandInfo instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <HandInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:id-val is deprecated.  Use qt_nuitrack_app-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'right_projection-val :lambda-list '(m))
(cl:defmethod right_projection-val ((m <HandInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:right_projection-val is deprecated.  Use qt_nuitrack_app-msg:right_projection instead.")
  (right_projection m))

(cl:ensure-generic-function 'right_real-val :lambda-list '(m))
(cl:defmethod right_real-val ((m <HandInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:right_real-val is deprecated.  Use qt_nuitrack_app-msg:right_real instead.")
  (right_real m))

(cl:ensure-generic-function 'right_click-val :lambda-list '(m))
(cl:defmethod right_click-val ((m <HandInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:right_click-val is deprecated.  Use qt_nuitrack_app-msg:right_click instead.")
  (right_click m))

(cl:ensure-generic-function 'right_pressure-val :lambda-list '(m))
(cl:defmethod right_pressure-val ((m <HandInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:right_pressure-val is deprecated.  Use qt_nuitrack_app-msg:right_pressure instead.")
  (right_pressure m))

(cl:ensure-generic-function 'left_projection-val :lambda-list '(m))
(cl:defmethod left_projection-val ((m <HandInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:left_projection-val is deprecated.  Use qt_nuitrack_app-msg:left_projection instead.")
  (left_projection m))

(cl:ensure-generic-function 'left_real-val :lambda-list '(m))
(cl:defmethod left_real-val ((m <HandInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:left_real-val is deprecated.  Use qt_nuitrack_app-msg:left_real instead.")
  (left_real m))

(cl:ensure-generic-function 'left_click-val :lambda-list '(m))
(cl:defmethod left_click-val ((m <HandInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:left_click-val is deprecated.  Use qt_nuitrack_app-msg:left_click instead.")
  (left_click m))

(cl:ensure-generic-function 'left_pressure-val :lambda-list '(m))
(cl:defmethod left_pressure-val ((m <HandInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_nuitrack_app-msg:left_pressure-val is deprecated.  Use qt_nuitrack_app-msg:left_pressure instead.")
  (left_pressure m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandInfo>) ostream)
  "Serializes a message object of type '<HandInfo>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'right_projection))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'right_projection))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'right_real))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'right_real))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_click) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'right_pressure)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'left_projection))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'left_projection))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'left_real))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'left_real))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left_click) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'left_pressure)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandInfo>) istream)
  "Deserializes a message object of type '<HandInfo>"
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
  (cl:setf (cl:slot-value msg 'right_projection) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'right_projection)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'right_real) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'right_real)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'right_click) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_pressure) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'left_projection) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'left_projection)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'left_real) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'left_real)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'left_click) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_pressure) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandInfo>)))
  "Returns string type for a message object of type '<HandInfo>"
  "qt_nuitrack_app/HandInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandInfo)))
  "Returns string type for a message object of type 'HandInfo"
  "qt_nuitrack_app/HandInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandInfo>)))
  "Returns md5sum for a message object of type '<HandInfo>"
  "ebda9d77d8f24f2a507d651777ac1949")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandInfo)))
  "Returns md5sum for a message object of type 'HandInfo"
  "ebda9d77d8f24f2a507d651777ac1949")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandInfo>)))
  "Returns full string definition for message of type '<HandInfo>"
  (cl:format cl:nil "#std_msgs/Header header~%int32 id~%#The normalized projective (x, y) coordinate of the right hand~%float32[] right_projection~%#The (x,y,z) coordinate of the right hand in the world system.~%float32[] right_real~%bool right_click~%int32 right_pressure~%#The normalized projective (x, y) coordinate of the left hand~%float32[] left_projection~%#The (x,y,z) coordinate of the left hand in the world system.~%float32[] left_real~%bool left_click~%int32 left_pressure~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandInfo)))
  "Returns full string definition for message of type 'HandInfo"
  (cl:format cl:nil "#std_msgs/Header header~%int32 id~%#The normalized projective (x, y) coordinate of the right hand~%float32[] right_projection~%#The (x,y,z) coordinate of the right hand in the world system.~%float32[] right_real~%bool right_click~%int32 right_pressure~%#The normalized projective (x, y) coordinate of the left hand~%float32[] left_projection~%#The (x,y,z) coordinate of the left hand in the world system.~%float32[] left_real~%bool left_click~%int32 left_pressure~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandInfo>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'right_projection) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'right_real) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'left_projection) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'left_real) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'HandInfo
    (cl:cons ':id (id msg))
    (cl:cons ':right_projection (right_projection msg))
    (cl:cons ':right_real (right_real msg))
    (cl:cons ':right_click (right_click msg))
    (cl:cons ':right_pressure (right_pressure msg))
    (cl:cons ':left_projection (left_projection msg))
    (cl:cons ':left_real (left_real msg))
    (cl:cons ':left_click (left_click msg))
    (cl:cons ':left_pressure (left_pressure msg))
))
