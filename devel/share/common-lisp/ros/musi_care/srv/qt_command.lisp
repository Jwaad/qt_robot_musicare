; Auto-generated. Do not edit!


(cl:in-package musi_care-srv)


;//! \htmlinclude qt_command-request.msg.html

(cl:defclass <qt_command-request> (roslisp-msg-protocol:ros-message)
  ((action_type
    :reader action_type
    :initarg :action_type
    :type cl:string
    :initform "")
   (action_content
    :reader action_content
    :initarg :action_content
    :type cl:string
    :initform "")
   (action_blocking
    :reader action_blocking
    :initarg :action_blocking
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass qt_command-request (<qt_command-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <qt_command-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'qt_command-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name musi_care-srv:<qt_command-request> is deprecated: use musi_care-srv:qt_command-request instead.")))

(cl:ensure-generic-function 'action_type-val :lambda-list '(m))
(cl:defmethod action_type-val ((m <qt_command-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader musi_care-srv:action_type-val is deprecated.  Use musi_care-srv:action_type instead.")
  (action_type m))

(cl:ensure-generic-function 'action_content-val :lambda-list '(m))
(cl:defmethod action_content-val ((m <qt_command-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader musi_care-srv:action_content-val is deprecated.  Use musi_care-srv:action_content instead.")
  (action_content m))

(cl:ensure-generic-function 'action_blocking-val :lambda-list '(m))
(cl:defmethod action_blocking-val ((m <qt_command-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader musi_care-srv:action_blocking-val is deprecated.  Use musi_care-srv:action_blocking instead.")
  (action_blocking m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <qt_command-request>) ostream)
  "Serializes a message object of type '<qt_command-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'action_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'action_type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'action_content))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'action_content))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'action_blocking) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <qt_command-request>) istream)
  "Deserializes a message object of type '<qt_command-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'action_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action_content) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'action_content) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'action_blocking) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<qt_command-request>)))
  "Returns string type for a service object of type '<qt_command-request>"
  "musi_care/qt_commandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'qt_command-request)))
  "Returns string type for a service object of type 'qt_command-request"
  "musi_care/qt_commandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<qt_command-request>)))
  "Returns md5sum for a message object of type '<qt_command-request>"
  "63c073458379b344b40c9cac0c2eacff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'qt_command-request)))
  "Returns md5sum for a message object of type 'qt_command-request"
  "63c073458379b344b40c9cac0c2eacff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<qt_command-request>)))
  "Returns full string definition for message of type '<qt_command-request>"
  (cl:format cl:nil "string action_type~%string action_content~%bool action_blocking~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'qt_command-request)))
  "Returns full string definition for message of type 'qt_command-request"
  (cl:format cl:nil "string action_type~%string action_content~%bool action_blocking~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <qt_command-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'action_type))
     4 (cl:length (cl:slot-value msg 'action_content))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <qt_command-request>))
  "Converts a ROS message object to a list"
  (cl:list 'qt_command-request
    (cl:cons ':action_type (action_type msg))
    (cl:cons ':action_content (action_content msg))
    (cl:cons ':action_blocking (action_blocking msg))
))
;//! \htmlinclude qt_command-response.msg.html

(cl:defclass <qt_command-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass qt_command-response (<qt_command-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <qt_command-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'qt_command-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name musi_care-srv:<qt_command-response> is deprecated: use musi_care-srv:qt_command-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <qt_command-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader musi_care-srv:status-val is deprecated.  Use musi_care-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <qt_command-response>) ostream)
  "Serializes a message object of type '<qt_command-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <qt_command-response>) istream)
  "Deserializes a message object of type '<qt_command-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<qt_command-response>)))
  "Returns string type for a service object of type '<qt_command-response>"
  "musi_care/qt_commandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'qt_command-response)))
  "Returns string type for a service object of type 'qt_command-response"
  "musi_care/qt_commandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<qt_command-response>)))
  "Returns md5sum for a message object of type '<qt_command-response>"
  "63c073458379b344b40c9cac0c2eacff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'qt_command-response)))
  "Returns md5sum for a message object of type 'qt_command-response"
  "63c073458379b344b40c9cac0c2eacff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<qt_command-response>)))
  "Returns full string definition for message of type '<qt_command-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'qt_command-response)))
  "Returns full string definition for message of type 'qt_command-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <qt_command-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <qt_command-response>))
  "Converts a ROS message object to a list"
  (cl:list 'qt_command-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'qt_command)))
  'qt_command-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'qt_command)))
  'qt_command-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'qt_command)))
  "Returns string type for a service object of type '<qt_command>"
  "musi_care/qt_command")