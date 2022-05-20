; Auto-generated. Do not edit!


(cl:in-package musi_care-msg)


;//! \htmlinclude SongData.msg.html

(cl:defclass <SongData> (roslisp-msg-protocol:ros-message)
  ((track_title
    :reader track_title
    :initarg :track_title
    :type cl:string
    :initform "")
   (track_elapsed_time
    :reader track_elapsed_time
    :initarg :track_elapsed_time
    :type cl:float
    :initform 0.0)
   (track_total_time
    :reader track_total_time
    :initarg :track_total_time
    :type cl:float
    :initform 0.0))
)

(cl:defclass SongData (<SongData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SongData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SongData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name musi_care-msg:<SongData> is deprecated: use musi_care-msg:SongData instead.")))

(cl:ensure-generic-function 'track_title-val :lambda-list '(m))
(cl:defmethod track_title-val ((m <SongData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader musi_care-msg:track_title-val is deprecated.  Use musi_care-msg:track_title instead.")
  (track_title m))

(cl:ensure-generic-function 'track_elapsed_time-val :lambda-list '(m))
(cl:defmethod track_elapsed_time-val ((m <SongData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader musi_care-msg:track_elapsed_time-val is deprecated.  Use musi_care-msg:track_elapsed_time instead.")
  (track_elapsed_time m))

(cl:ensure-generic-function 'track_total_time-val :lambda-list '(m))
(cl:defmethod track_total_time-val ((m <SongData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader musi_care-msg:track_total_time-val is deprecated.  Use musi_care-msg:track_total_time instead.")
  (track_total_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SongData>) ostream)
  "Serializes a message object of type '<SongData>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'track_title))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'track_title))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'track_elapsed_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'track_total_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SongData>) istream)
  "Deserializes a message object of type '<SongData>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'track_title) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'track_title) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'track_elapsed_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'track_total_time) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SongData>)))
  "Returns string type for a message object of type '<SongData>"
  "musi_care/SongData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SongData)))
  "Returns string type for a message object of type 'SongData"
  "musi_care/SongData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SongData>)))
  "Returns md5sum for a message object of type '<SongData>"
  "82324e20ca3e42f8d6a8f9e9503524ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SongData)))
  "Returns md5sum for a message object of type 'SongData"
  "82324e20ca3e42f8d6a8f9e9503524ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SongData>)))
  "Returns full string definition for message of type '<SongData>"
  (cl:format cl:nil "string track_title~%float32 track_elapsed_time~%float32 track_total_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SongData)))
  "Returns full string definition for message of type 'SongData"
  (cl:format cl:nil "string track_title~%float32 track_elapsed_time~%float32 track_total_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SongData>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'track_title))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SongData>))
  "Converts a ROS message object to a list"
  (cl:list 'SongData
    (cl:cons ':track_title (track_title msg))
    (cl:cons ':track_elapsed_time (track_elapsed_time msg))
    (cl:cons ':track_total_time (track_total_time msg))
))
