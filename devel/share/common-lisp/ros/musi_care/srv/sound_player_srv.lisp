; Auto-generated. Do not edit!


(cl:in-package musi_care-srv)


;//! \htmlinclude sound_player_srv-request.msg.html

(cl:defclass <sound_player_srv-request> (roslisp-msg-protocol:ros-message)
  ((operation
    :reader operation
    :initarg :operation
    :type cl:string
    :initform "")
   (operation_data_1
    :reader operation_data_1
    :initarg :operation_data_1
    :type cl:string
    :initform "")
   (operation_data_2
    :reader operation_data_2
    :initarg :operation_data_2
    :type cl:float
    :initform 0.0))
)

(cl:defclass sound_player_srv-request (<sound_player_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sound_player_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sound_player_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name musi_care-srv:<sound_player_srv-request> is deprecated: use musi_care-srv:sound_player_srv-request instead.")))

(cl:ensure-generic-function 'operation-val :lambda-list '(m))
(cl:defmethod operation-val ((m <sound_player_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader musi_care-srv:operation-val is deprecated.  Use musi_care-srv:operation instead.")
  (operation m))

(cl:ensure-generic-function 'operation_data_1-val :lambda-list '(m))
(cl:defmethod operation_data_1-val ((m <sound_player_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader musi_care-srv:operation_data_1-val is deprecated.  Use musi_care-srv:operation_data_1 instead.")
  (operation_data_1 m))

(cl:ensure-generic-function 'operation_data_2-val :lambda-list '(m))
(cl:defmethod operation_data_2-val ((m <sound_player_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader musi_care-srv:operation_data_2-val is deprecated.  Use musi_care-srv:operation_data_2 instead.")
  (operation_data_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sound_player_srv-request>) ostream)
  "Serializes a message object of type '<sound_player_srv-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'operation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'operation))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'operation_data_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'operation_data_1))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'operation_data_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sound_player_srv-request>) istream)
  "Deserializes a message object of type '<sound_player_srv-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'operation) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'operation) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'operation_data_1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'operation_data_1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'operation_data_2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sound_player_srv-request>)))
  "Returns string type for a service object of type '<sound_player_srv-request>"
  "musi_care/sound_player_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sound_player_srv-request)))
  "Returns string type for a service object of type 'sound_player_srv-request"
  "musi_care/sound_player_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sound_player_srv-request>)))
  "Returns md5sum for a message object of type '<sound_player_srv-request>"
  "2a3bef821864039a35e20805c207d7a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sound_player_srv-request)))
  "Returns md5sum for a message object of type 'sound_player_srv-request"
  "2a3bef821864039a35e20805c207d7a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sound_player_srv-request>)))
  "Returns full string definition for message of type '<sound_player_srv-request>"
  (cl:format cl:nil "string operation~%string operation_data_1~%float32 operation_data_2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sound_player_srv-request)))
  "Returns full string definition for message of type 'sound_player_srv-request"
  (cl:format cl:nil "string operation~%string operation_data_1~%float32 operation_data_2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sound_player_srv-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'operation))
     4 (cl:length (cl:slot-value msg 'operation_data_1))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sound_player_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'sound_player_srv-request
    (cl:cons ':operation (operation msg))
    (cl:cons ':operation_data_1 (operation_data_1 msg))
    (cl:cons ':operation_data_2 (operation_data_2 msg))
))
;//! \htmlinclude sound_player_srv-response.msg.html

(cl:defclass <sound_player_srv-response> (roslisp-msg-protocol:ros-message)
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
    :initform 0.0)
   (status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass sound_player_srv-response (<sound_player_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sound_player_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sound_player_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name musi_care-srv:<sound_player_srv-response> is deprecated: use musi_care-srv:sound_player_srv-response instead.")))

(cl:ensure-generic-function 'track_title-val :lambda-list '(m))
(cl:defmethod track_title-val ((m <sound_player_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader musi_care-srv:track_title-val is deprecated.  Use musi_care-srv:track_title instead.")
  (track_title m))

(cl:ensure-generic-function 'track_elapsed_time-val :lambda-list '(m))
(cl:defmethod track_elapsed_time-val ((m <sound_player_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader musi_care-srv:track_elapsed_time-val is deprecated.  Use musi_care-srv:track_elapsed_time instead.")
  (track_elapsed_time m))

(cl:ensure-generic-function 'track_total_time-val :lambda-list '(m))
(cl:defmethod track_total_time-val ((m <sound_player_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader musi_care-srv:track_total_time-val is deprecated.  Use musi_care-srv:track_total_time instead.")
  (track_total_time m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <sound_player_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader musi_care-srv:status-val is deprecated.  Use musi_care-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sound_player_srv-response>) ostream)
  "Serializes a message object of type '<sound_player_srv-response>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sound_player_srv-response>) istream)
  "Deserializes a message object of type '<sound_player_srv-response>"
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
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sound_player_srv-response>)))
  "Returns string type for a service object of type '<sound_player_srv-response>"
  "musi_care/sound_player_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sound_player_srv-response)))
  "Returns string type for a service object of type 'sound_player_srv-response"
  "musi_care/sound_player_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sound_player_srv-response>)))
  "Returns md5sum for a message object of type '<sound_player_srv-response>"
  "2a3bef821864039a35e20805c207d7a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sound_player_srv-response)))
  "Returns md5sum for a message object of type 'sound_player_srv-response"
  "2a3bef821864039a35e20805c207d7a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sound_player_srv-response>)))
  "Returns full string definition for message of type '<sound_player_srv-response>"
  (cl:format cl:nil "string track_title~%float32 track_elapsed_time~%float32 track_total_time~%bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sound_player_srv-response)))
  "Returns full string definition for message of type 'sound_player_srv-response"
  (cl:format cl:nil "string track_title~%float32 track_elapsed_time~%float32 track_total_time~%bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sound_player_srv-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'track_title))
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sound_player_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'sound_player_srv-response
    (cl:cons ':track_title (track_title msg))
    (cl:cons ':track_elapsed_time (track_elapsed_time msg))
    (cl:cons ':track_total_time (track_total_time msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'sound_player_srv)))
  'sound_player_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'sound_player_srv)))
  'sound_player_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sound_player_srv)))
  "Returns string type for a service object of type '<sound_player_srv>"
  "musi_care/sound_player_srv")