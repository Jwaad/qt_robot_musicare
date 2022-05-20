;; Auto-generated. Do not edit!


(when (boundp 'qt_nuitrack_app::suspend)
  (if (not (find-package "QT_NUITRACK_APP"))
    (make-package "QT_NUITRACK_APP"))
  (shadow 'suspend (find-package "QT_NUITRACK_APP")))
(unless (find-package "QT_NUITRACK_APP::SUSPEND")
  (make-package "QT_NUITRACK_APP::SUSPEND"))
(unless (find-package "QT_NUITRACK_APP::SUSPENDREQUEST")
  (make-package "QT_NUITRACK_APP::SUSPENDREQUEST"))
(unless (find-package "QT_NUITRACK_APP::SUSPENDRESPONSE")
  (make-package "QT_NUITRACK_APP::SUSPENDRESPONSE"))

(in-package "ROS")





(defclass qt_nuitrack_app::suspendRequest
  :super ros::object
  :slots (_flag ))

(defmethod qt_nuitrack_app::suspendRequest
  (:init
   (&key
    ((:flag __flag) nil)
    )
   (send-super :init)
   (setq _flag __flag)
   self)
  (:flag
   (&optional (__flag :null))
   (if (not (eq __flag :null)) (setq _flag __flag)) _flag)
  (:serialization-length
   ()
   (+
    ;; bool _flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _flag
       (if _flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _flag
     (setq _flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass qt_nuitrack_app::suspendResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_nuitrack_app::suspendResponse
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

(defclass qt_nuitrack_app::suspend
  :super ros::object
  :slots ())

(setf (get qt_nuitrack_app::suspend :md5sum-) "1ec93149d55d1527cff5331cb7a5fe0a")
(setf (get qt_nuitrack_app::suspend :datatype-) "qt_nuitrack_app/suspend")
(setf (get qt_nuitrack_app::suspend :request) qt_nuitrack_app::suspendRequest)
(setf (get qt_nuitrack_app::suspend :response) qt_nuitrack_app::suspendResponse)

(defmethod qt_nuitrack_app::suspendRequest
  (:response () (instance qt_nuitrack_app::suspendResponse :init)))

(setf (get qt_nuitrack_app::suspendRequest :md5sum-) "1ec93149d55d1527cff5331cb7a5fe0a")
(setf (get qt_nuitrack_app::suspendRequest :datatype-) "qt_nuitrack_app/suspendRequest")
(setf (get qt_nuitrack_app::suspendRequest :definition-)
      "
bool flag
---
bool status

")

(setf (get qt_nuitrack_app::suspendResponse :md5sum-) "1ec93149d55d1527cff5331cb7a5fe0a")
(setf (get qt_nuitrack_app::suspendResponse :datatype-) "qt_nuitrack_app/suspendResponse")
(setf (get qt_nuitrack_app::suspendResponse :definition-)
      "
bool flag
---
bool status

")



(provide :qt_nuitrack_app/suspend "1ec93149d55d1527cff5331cb7a5fe0a")


