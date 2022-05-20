;; Auto-generated. Do not edit!


(when (boundp 'qt_nuitrack_app::Gestures)
  (if (not (find-package "QT_NUITRACK_APP"))
    (make-package "QT_NUITRACK_APP"))
  (shadow 'Gestures (find-package "QT_NUITRACK_APP")))
(unless (find-package "QT_NUITRACK_APP::GESTURES")
  (make-package "QT_NUITRACK_APP::GESTURES"))

(in-package "ROS")
;;//! \htmlinclude Gestures.msg.html


(defclass qt_nuitrack_app::Gestures
  :super ros::object
  :slots (_gestures ))

(defmethod qt_nuitrack_app::Gestures
  (:init
   (&key
    ((:gestures __gestures) ())
    )
   (send-super :init)
   (setq _gestures __gestures)
   self)
  (:gestures
   (&rest __gestures)
   (if (keywordp (car __gestures))
       (send* _gestures __gestures)
     (progn
       (if __gestures (setq _gestures (car __gestures)))
       _gestures)))
  (:serialization-length
   ()
   (+
    ;; qt_nuitrack_app/GestureInfo[] _gestures
    (apply #'+ (send-all _gestures :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; qt_nuitrack_app/GestureInfo[] _gestures
     (write-long (length _gestures) s)
     (dolist (elem _gestures)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; qt_nuitrack_app/GestureInfo[] _gestures
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gestures (let (r) (dotimes (i n) (push (instance qt_nuitrack_app::GestureInfo :init) r)) r))
     (dolist (elem- _gestures)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get qt_nuitrack_app::Gestures :md5sum-) "cb33e599df6d94bfd33170149d8d8cba")
(setf (get qt_nuitrack_app::Gestures :datatype-) "qt_nuitrack_app/Gestures")
(setf (get qt_nuitrack_app::Gestures :definition-)
      "#std_msgs/Header header
GestureInfo[] gestures

================================================================================
MSG: qt_nuitrack_app/GestureInfo
#std_msgs/Header header
int32 id
string name

")



(provide :qt_nuitrack_app/Gestures "cb33e599df6d94bfd33170149d8d8cba")


