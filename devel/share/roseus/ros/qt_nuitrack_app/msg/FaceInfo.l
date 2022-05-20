;; Auto-generated. Do not edit!


(when (boundp 'qt_nuitrack_app::FaceInfo)
  (if (not (find-package "QT_NUITRACK_APP"))
    (make-package "QT_NUITRACK_APP"))
  (shadow 'FaceInfo (find-package "QT_NUITRACK_APP")))
(unless (find-package "QT_NUITRACK_APP::FACEINFO")
  (make-package "QT_NUITRACK_APP::FACEINFO"))

(in-package "ROS")
;;//! \htmlinclude FaceInfo.msg.html


(defclass qt_nuitrack_app::FaceInfo
  :super ros::object
  :slots (_id _gender _age_years _age_type _emotion_neutral _emotion_angry _emotion_happy _emotion_surprise _rectangle _left_eye _right_eye _angles ))

(defmethod qt_nuitrack_app::FaceInfo
  (:init
   (&key
    ((:id __id) 0)
    ((:gender __gender) "")
    ((:age_years __age_years) 0)
    ((:age_type __age_type) "")
    ((:emotion_neutral __emotion_neutral) 0.0)
    ((:emotion_angry __emotion_angry) 0.0)
    ((:emotion_happy __emotion_happy) 0.0)
    ((:emotion_surprise __emotion_surprise) 0.0)
    ((:rectangle __rectangle) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:left_eye __left_eye) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:right_eye __right_eye) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:angles __angles) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _gender (string __gender))
   (setq _age_years (round __age_years))
   (setq _age_type (string __age_type))
   (setq _emotion_neutral (float __emotion_neutral))
   (setq _emotion_angry (float __emotion_angry))
   (setq _emotion_happy (float __emotion_happy))
   (setq _emotion_surprise (float __emotion_surprise))
   (setq _rectangle __rectangle)
   (setq _left_eye __left_eye)
   (setq _right_eye __right_eye)
   (setq _angles __angles)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:gender
   (&optional __gender)
   (if __gender (setq _gender __gender)) _gender)
  (:age_years
   (&optional __age_years)
   (if __age_years (setq _age_years __age_years)) _age_years)
  (:age_type
   (&optional __age_type)
   (if __age_type (setq _age_type __age_type)) _age_type)
  (:emotion_neutral
   (&optional __emotion_neutral)
   (if __emotion_neutral (setq _emotion_neutral __emotion_neutral)) _emotion_neutral)
  (:emotion_angry
   (&optional __emotion_angry)
   (if __emotion_angry (setq _emotion_angry __emotion_angry)) _emotion_angry)
  (:emotion_happy
   (&optional __emotion_happy)
   (if __emotion_happy (setq _emotion_happy __emotion_happy)) _emotion_happy)
  (:emotion_surprise
   (&optional __emotion_surprise)
   (if __emotion_surprise (setq _emotion_surprise __emotion_surprise)) _emotion_surprise)
  (:rectangle
   (&optional __rectangle)
   (if __rectangle (setq _rectangle __rectangle)) _rectangle)
  (:left_eye
   (&optional __left_eye)
   (if __left_eye (setq _left_eye __left_eye)) _left_eye)
  (:right_eye
   (&optional __right_eye)
   (if __right_eye (setq _right_eye __right_eye)) _right_eye)
  (:angles
   (&optional __angles)
   (if __angles (setq _angles __angles)) _angles)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; string _gender
    4 (length _gender)
    ;; int32 _age_years
    4
    ;; string _age_type
    4 (length _age_type)
    ;; float64 _emotion_neutral
    8
    ;; float64 _emotion_angry
    8
    ;; float64 _emotion_happy
    8
    ;; float64 _emotion_surprise
    8
    ;; float64[] _rectangle
    (* 8    (length _rectangle)) 4
    ;; float64[] _left_eye
    (* 8    (length _left_eye)) 4
    ;; float64[] _right_eye
    (* 8    (length _right_eye)) 4
    ;; float64[] _angles
    (* 8    (length _angles)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; string _gender
       (write-long (length _gender) s) (princ _gender s)
     ;; int32 _age_years
       (write-long _age_years s)
     ;; string _age_type
       (write-long (length _age_type) s) (princ _age_type s)
     ;; float64 _emotion_neutral
       (sys::poke _emotion_neutral (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _emotion_angry
       (sys::poke _emotion_angry (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _emotion_happy
       (sys::poke _emotion_happy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _emotion_surprise
       (sys::poke _emotion_surprise (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _rectangle
     (write-long (length _rectangle) s)
     (dotimes (i (length _rectangle))
       (sys::poke (elt _rectangle i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _left_eye
     (write-long (length _left_eye) s)
     (dotimes (i (length _left_eye))
       (sys::poke (elt _left_eye i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _right_eye
     (write-long (length _right_eye) s)
     (dotimes (i (length _right_eye))
       (sys::poke (elt _right_eye i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _angles
     (write-long (length _angles) s)
     (dotimes (i (length _angles))
       (sys::poke (elt _angles i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _gender
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gender (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _age_years
     (setq _age_years (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _age_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _age_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _emotion_neutral
     (setq _emotion_neutral (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _emotion_angry
     (setq _emotion_angry (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _emotion_happy
     (setq _emotion_happy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _emotion_surprise
     (setq _emotion_surprise (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _rectangle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rectangle (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _rectangle i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _left_eye
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _left_eye (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _left_eye i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _right_eye
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _right_eye (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _right_eye i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _angles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angles (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _angles i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get qt_nuitrack_app::FaceInfo :md5sum-) "b210720ecbe10f5a0a7a3911214f766a")
(setf (get qt_nuitrack_app::FaceInfo :datatype-) "qt_nuitrack_app/FaceInfo")
(setf (get qt_nuitrack_app::FaceInfo :definition-)
      "#std_msgs/Header header
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



(provide :qt_nuitrack_app/FaceInfo "b210720ecbe10f5a0a7a3911214f766a")


