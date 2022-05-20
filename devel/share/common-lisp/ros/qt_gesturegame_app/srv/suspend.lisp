; Auto-generated. Do not edit!


(cl:in-package qt_gesturegame_app-srv)


;//! \htmlinclude suspend-request.msg.html

(cl:defclass <suspend-request> (roslisp-msg-protocol:ros-message)
  ((flag
    :reader flag
    :initarg :flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass suspend-request (<suspend-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <suspend-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'suspend-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_gesturegame_app-srv:<suspend-request> is deprecated: use qt_gesturegame_app-srv:suspend-request instead.")))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <suspend-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesturegame_app-srv:flag-val is deprecated.  Use qt_gesturegame_app-srv:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <suspend-request>) ostream)
  "Serializes a message object of type '<suspend-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <suspend-request>) istream)
  "Deserializes a message object of type '<suspend-request>"
    (cl:setf (cl:slot-value msg 'flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<suspend-request>)))
  "Returns string type for a service object of type '<suspend-request>"
  "qt_gesturegame_app/suspendRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'suspend-request)))
  "Returns string type for a service object of type 'suspend-request"
  "qt_gesturegame_app/suspendRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<suspend-request>)))
  "Returns md5sum for a message object of type '<suspend-request>"
  "1ec93149d55d1527cff5331cb7a5fe0a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'suspend-request)))
  "Returns md5sum for a message object of type 'suspend-request"
  "1ec93149d55d1527cff5331cb7a5fe0a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<suspend-request>)))
  "Returns full string definition for message of type '<suspend-request>"
  (cl:format cl:nil "~%bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'suspend-request)))
  "Returns full string definition for message of type 'suspend-request"
  (cl:format cl:nil "~%bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <suspend-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <suspend-request>))
  "Converts a ROS message object to a list"
  (cl:list 'suspend-request
    (cl:cons ':flag (flag msg))
))
;//! \htmlinclude suspend-response.msg.html

(cl:defclass <suspend-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass suspend-response (<suspend-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <suspend-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'suspend-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_gesturegame_app-srv:<suspend-response> is deprecated: use qt_gesturegame_app-srv:suspend-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <suspend-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesturegame_app-srv:status-val is deprecated.  Use qt_gesturegame_app-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <suspend-response>) ostream)
  "Serializes a message object of type '<suspend-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <suspend-response>) istream)
  "Deserializes a message object of type '<suspend-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<suspend-response>)))
  "Returns string type for a service object of type '<suspend-response>"
  "qt_gesturegame_app/suspendResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'suspend-response)))
  "Returns string type for a service object of type 'suspend-response"
  "qt_gesturegame_app/suspendResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<suspend-response>)))
  "Returns md5sum for a message object of type '<suspend-response>"
  "1ec93149d55d1527cff5331cb7a5fe0a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'suspend-response)))
  "Returns md5sum for a message object of type 'suspend-response"
  "1ec93149d55d1527cff5331cb7a5fe0a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<suspend-response>)))
  "Returns full string definition for message of type '<suspend-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'suspend-response)))
  "Returns full string definition for message of type 'suspend-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <suspend-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <suspend-response>))
  "Converts a ROS message object to a list"
  (cl:list 'suspend-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'suspend)))
  'suspend-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'suspend)))
  'suspend-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'suspend)))
  "Returns string type for a service object of type '<suspend>"
  "qt_gesturegame_app/suspend")