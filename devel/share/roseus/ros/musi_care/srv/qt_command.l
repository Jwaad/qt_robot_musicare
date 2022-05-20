;; Auto-generated. Do not edit!


(when (boundp 'musi_care::qt_command)
  (if (not (find-package "MUSI_CARE"))
    (make-package "MUSI_CARE"))
  (shadow 'qt_command (find-package "MUSI_CARE")))
(unless (find-package "MUSI_CARE::QT_COMMAND")
  (make-package "MUSI_CARE::QT_COMMAND"))
(unless (find-package "MUSI_CARE::QT_COMMANDREQUEST")
  (make-package "MUSI_CARE::QT_COMMANDREQUEST"))
(unless (find-package "MUSI_CARE::QT_COMMANDRESPONSE")
  (make-package "MUSI_CARE::QT_COMMANDRESPONSE"))

(in-package "ROS")





(defclass musi_care::qt_commandRequest
  :super ros::object
  :slots (_action_type _action_content _action_blocking ))

(defmethod musi_care::qt_commandRequest
  (:init
   (&key
    ((:action_type __action_type) "")
    ((:action_content __action_content) "")
    ((:action_blocking __action_blocking) nil)
    )
   (send-super :init)
   (setq _action_type (string __action_type))
   (setq _action_content (string __action_content))
   (setq _action_blocking __action_blocking)
   self)
  (:action_type
   (&optional __action_type)
   (if __action_type (setq _action_type __action_type)) _action_type)
  (:action_content
   (&optional __action_content)
   (if __action_content (setq _action_content __action_content)) _action_content)
  (:action_blocking
   (&optional (__action_blocking :null))
   (if (not (eq __action_blocking :null)) (setq _action_blocking __action_blocking)) _action_blocking)
  (:serialization-length
   ()
   (+
    ;; string _action_type
    4 (length _action_type)
    ;; string _action_content
    4 (length _action_content)
    ;; bool _action_blocking
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _action_type
       (write-long (length _action_type) s) (princ _action_type s)
     ;; string _action_content
       (write-long (length _action_content) s) (princ _action_content s)
     ;; bool _action_blocking
       (if _action_blocking (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _action_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _action_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _action_content
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _action_content (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _action_blocking
     (setq _action_blocking (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass musi_care::qt_commandResponse
  :super ros::object
  :slots (_status ))

(defmethod musi_care::qt_commandResponse
  (:init
   (&key
    ((:status __status) nil)
    )
   (send-super :init)
   (setq _status __status)
   self)
  (:status
   (&optional (__status :null))
   (if (not (eq __status :null)) (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; bool _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass musi_care::qt_command
  :super ros::object
  :slots ())

(setf (get musi_care::qt_command :md5sum-) "63c073458379b344b40c9cac0c2eacff")
(setf (get musi_care::qt_command :datatype-) "musi_care/qt_command")
(setf (get musi_care::qt_command :request) musi_care::qt_commandRequest)
(setf (get musi_care::qt_command :response) musi_care::qt_commandResponse)

(defmethod musi_care::qt_commandRequest
  (:response () (instance musi_care::qt_commandResponse :init)))

(setf (get musi_care::qt_commandRequest :md5sum-) "63c073458379b344b40c9cac0c2eacff")
(setf (get musi_care::qt_commandRequest :datatype-) "musi_care/qt_commandRequest")
(setf (get musi_care::qt_commandRequest :definition-)
      "string action_type
string action_content
bool action_blocking
---
bool status

")

(setf (get musi_care::qt_commandResponse :md5sum-) "63c073458379b344b40c9cac0c2eacff")
(setf (get musi_care::qt_commandResponse :datatype-) "musi_care/qt_commandResponse")
(setf (get musi_care::qt_commandResponse :definition-)
      "string action_type
string action_content
bool action_blocking
---
bool status

")



(provide :musi_care/qt_command "63c073458379b344b40c9cac0c2eacff")


