;; Auto-generated. Do not edit!


(when (boundp 'qt_nuitrack_app::SkeletonInfo)
  (if (not (find-package "QT_NUITRACK_APP"))
    (make-package "QT_NUITRACK_APP"))
  (shadow 'SkeletonInfo (find-package "QT_NUITRACK_APP")))
(unless (find-package "QT_NUITRACK_APP::SKELETONINFO")
  (make-package "QT_NUITRACK_APP::SKELETONINFO"))

(in-package "ROS")
;;//! \htmlinclude SkeletonInfo.msg.html


(defclass qt_nuitrack_app::SkeletonInfo
  :super ros::object
  :slots (_id _joints ))

(defmethod qt_nuitrack_app::SkeletonInfo
  (:init
   (&key
    ((:id __id) 0)
    ((:joints __joints) ())
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _joints __joints)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:joints
   (&rest __joints)
   (if (keywordp (car __joints))
       (send* _joints __joints)
     (progn
       (if __joints (setq _joints (car __joints)))
       _joints)))
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; qt_nuitrack_app/JointInfo[] _joints
    (apply #'+ (send-all _joints :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; qt_nuitrack_app/JointInfo[] _joints
     (write-long (length _joints) s)
     (dolist (elem _joints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; qt_nuitrack_app/JointInfo[] _joints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joints (let (r) (dotimes (i n) (push (instance qt_nuitrack_app::JointInfo :init) r)) r))
     (dolist (elem- _joints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get qt_nuitrack_app::SkeletonInfo :md5sum-) "ac75876d5ead18b4ad4603a6b5b71a97")
(setf (get qt_nuitrack_app::SkeletonInfo :datatype-) "qt_nuitrack_app/SkeletonInfo")
(setf (get qt_nuitrack_app::SkeletonInfo :definition-)
      "#std_msgs/Header header
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



(provide :qt_nuitrack_app/SkeletonInfo "ac75876d5ead18b4ad4603a6b5b71a97")


