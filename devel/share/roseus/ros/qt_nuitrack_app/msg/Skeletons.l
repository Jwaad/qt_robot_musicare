;; Auto-generated. Do not edit!


(when (boundp 'qt_nuitrack_app::Skeletons)
  (if (not (find-package "QT_NUITRACK_APP"))
    (make-package "QT_NUITRACK_APP"))
  (shadow 'Skeletons (find-package "QT_NUITRACK_APP")))
(unless (find-package "QT_NUITRACK_APP::SKELETONS")
  (make-package "QT_NUITRACK_APP::SKELETONS"))

(in-package "ROS")
;;//! \htmlinclude Skeletons.msg.html


(defclass qt_nuitrack_app::Skeletons
  :super ros::object
  :slots (_skeletons ))

(defmethod qt_nuitrack_app::Skeletons
  (:init
   (&key
    ((:skeletons __skeletons) ())
    )
   (send-super :init)
   (setq _skeletons __skeletons)
   self)
  (:skeletons
   (&rest __skeletons)
   (if (keywordp (car __skeletons))
       (send* _skeletons __skeletons)
     (progn
       (if __skeletons (setq _skeletons (car __skeletons)))
       _skeletons)))
  (:serialization-length
   ()
   (+
    ;; qt_nuitrack_app/SkeletonInfo[] _skeletons
    (apply #'+ (send-all _skeletons :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; qt_nuitrack_app/SkeletonInfo[] _skeletons
     (write-long (length _skeletons) s)
     (dolist (elem _skeletons)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; qt_nuitrack_app/SkeletonInfo[] _skeletons
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _skeletons (let (r) (dotimes (i n) (push (instance qt_nuitrack_app::SkeletonInfo :init) r)) r))
     (dolist (elem- _skeletons)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get qt_nuitrack_app::Skeletons :md5sum-) "c9e6a0abe2b44381a0de593ee0c2c308")
(setf (get qt_nuitrack_app::Skeletons :datatype-) "qt_nuitrack_app/Skeletons")
(setf (get qt_nuitrack_app::Skeletons :definition-)
      "#std_msgs/Header header
SkeletonInfo[] skeletons

================================================================================
MSG: qt_nuitrack_app/SkeletonInfo
#std_msgs/Header header
int32 id
JointInfo[] joints 

================================================================================
MSG: qt_nuitrack_app/JointInfo
#std_msgs/Header header
uint8 type
float32 confidence
float32[] real
float32[] projection
float32[] orientation

")



(provide :qt_nuitrack_app/Skeletons "c9e6a0abe2b44381a0de593ee0c2c308")


