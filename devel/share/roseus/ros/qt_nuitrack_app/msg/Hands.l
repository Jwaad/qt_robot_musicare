;; Auto-generated. Do not edit!


(when (boundp 'qt_nuitrack_app::Hands)
  (if (not (find-package "QT_NUITRACK_APP"))
    (make-package "QT_NUITRACK_APP"))
  (shadow 'Hands (find-package "QT_NUITRACK_APP")))
(unless (find-package "QT_NUITRACK_APP::HANDS")
  (make-package "QT_NUITRACK_APP::HANDS"))

(in-package "ROS")
;;//! \htmlinclude Hands.msg.html


(defclass qt_nuitrack_app::Hands
  :super ros::object
  :slots (_hands ))

(defmethod qt_nuitrack_app::Hands
  (:init
   (&key
    ((:hands __hands) ())
    )
   (send-super :init)
   (setq _hands __hands)
   self)
  (:hands
   (&rest __hands)
   (if (keywordp (car __hands))
       (send* _hands __hands)
     (progn
       (if __hands (setq _hands (car __hands)))
       _hands)))
  (:serialization-length
   ()
   (+
    ;; qt_nuitrack_app/HandInfo[] _hands
    (apply #'+ (send-all _hands :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; qt_nuitrack_app/HandInfo[] _hands
     (write-long (length _hands) s)
     (dolist (elem _hands)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; qt_nuitrack_app/HandInfo[] _hands
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _hands (let (r) (dotimes (i n) (push (instance qt_nuitrack_app::HandInfo :init) r)) r))
     (dolist (elem- _hands)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get qt_nuitrack_app::Hands :md5sum-) "633d76b336567f0906335e6cf0195299")
(setf (get qt_nuitrack_app::Hands :datatype-) "qt_nuitrack_app/Hands")
(setf (get qt_nuitrack_app::Hands :definition-)
      "#std_msgs/Header header
HandInfo[] hands

================================================================================
MSG: qt_nuitrack_app/HandInfo
#std_msgs/Header header
int32 id
#The normalized projective (x, y) coordinate of the right hand
float32[] right_projection
#The (x,y,z) coordinate of the right hand in the world system.
float32[] right_real
bool right_click
int32 right_pressure
#The normalized projective (x, y) coordinate of the left hand
float32[] left_projection
#The (x,y,z) coordinate of the left hand in the world system.
float32[] left_real
bool left_click
int32 left_pressure

")



(provide :qt_nuitrack_app/Hands "633d76b336567f0906335e6cf0195299")


