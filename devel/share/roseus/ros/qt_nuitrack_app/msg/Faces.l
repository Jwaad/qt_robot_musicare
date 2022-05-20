;; Auto-generated. Do not edit!


(when (boundp 'qt_nuitrack_app::Faces)
  (if (not (find-package "QT_NUITRACK_APP"))
    (make-package "QT_NUITRACK_APP"))
  (shadow 'Faces (find-package "QT_NUITRACK_APP")))
(unless (find-package "QT_NUITRACK_APP::FACES")
  (make-package "QT_NUITRACK_APP::FACES"))

(in-package "ROS")
;;//! \htmlinclude Faces.msg.html


(defclass qt_nuitrack_app::Faces
  :super ros::object
  :slots (_faces ))

(defmethod qt_nuitrack_app::Faces
  (:init
   (&key
    ((:faces __faces) ())
    )
   (send-super :init)
   (setq _faces __faces)
   self)
  (:faces
   (&rest __faces)
   (if (keywordp (car __faces))
       (send* _faces __faces)
     (progn
       (if __faces (setq _faces (car __faces)))
       _faces)))
  (:serialization-length
   ()
   (+
    ;; qt_nuitrack_app/FaceInfo[] _faces
    (apply #'+ (send-all _faces :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; qt_nuitrack_app/FaceInfo[] _faces
     (write-long (length _faces) s)
     (dolist (elem _faces)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; qt_nuitrack_app/FaceInfo[] _faces
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _faces (let (r) (dotimes (i n) (push (instance qt_nuitrack_app::FaceInfo :init) r)) r))
     (dolist (elem- _faces)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get qt_nuitrack_app::Faces :md5sum-) "20dfb5db1e90db4dafd45bc663893505")
(setf (get qt_nuitrack_app::Faces :datatype-) "qt_nuitrack_app/Faces")
(setf (get qt_nuitrack_app::Faces :definition-)
      "#std_msgs/Header header
FaceInfo[] faces

================================================================================
MSG: qt_nuitrack_app/FaceInfo
#std_msgs/Header header
int32 id
string gender
int32 age_years
string age_type
float64 emotion_neutral
float64 emotion_angry
float64 emotion_happy
float64 emotion_surprise
#normalized screen coordinates of a face rectangle in the image (x, y, w, h)
float64[] rectangle
#normalized coordinates of the center of a person’s left eye (x,y)
float64[] left_eye
#normalized coordinates of the center of a person’s right eye (x,y)
float64[] right_eye
#face orientation angles in degrees (yaw, pitch, roll)
float64[] angles

")



(provide :qt_nuitrack_app/Faces "20dfb5db1e90db4dafd45bc663893505")


