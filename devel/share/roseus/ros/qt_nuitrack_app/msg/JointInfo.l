;; Auto-generated. Do not edit!


(when (boundp 'qt_nuitrack_app::JointInfo)
  (if (not (find-package "QT_NUITRACK_APP"))
    (make-package "QT_NUITRACK_APP"))
  (shadow 'JointInfo (find-package "QT_NUITRACK_APP")))
(unless (find-package "QT_NUITRACK_APP::JOINTINFO")
  (make-package "QT_NUITRACK_APP::JOINTINFO"))

(in-package "ROS")
;;//! \htmlinclude JointInfo.msg.html


(defclass qt_nuitrack_app::JointInfo
  :super ros::object
  :slots (_type _confidence _real _projection _orientation ))

(defmethod qt_nuitrack_app::JointInfo
  (:init
   (&key
    ((:type __type) 0)
    ((:confidence __confidence) 0.0)
    ((:real __real) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:projection __projection) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:orientation __orientation) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _confidence (float __confidence))
   (setq _real __real)
   (setq _projection __projection)
   (setq _orientation __orientation)
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:real
   (&optional __real)
   (if __real (setq _real __real)) _real)
  (:projection
   (&optional __projection)
   (if __projection (setq _projection __projection)) _projection)
  (:orientation
   (&optional __orientation)
   (if __orientation (setq _orientation __orientation)) _orientation)
  (:serialization-length
   ()
   (+
    ;; uint8 _type
    1
    ;; float32 _confidence
    4
    ;; float32[] _real
    (* 4    (length _real)) 4
    ;; float32[] _projection
    (* 4    (length _projection)) 4
    ;; float32[] _orientation
    (* 4    (length _orientation)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _type
       (write-byte _type s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _real
     (write-long (length _real) s)
     (dotimes (i (length _real))
       (sys::poke (elt _real i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _projection
     (write-long (length _projection) s)
     (dotimes (i (length _projection))
       (sys::poke (elt _projection i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _orientation
     (write-long (length _orientation) s)
     (dotimes (i (length _orientation))
       (sys::poke (elt _orientation i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _real
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _real (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _real i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _projection
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _projection (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _projection i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _orientation
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _orientation (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _orientation i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get qt_nuitrack_app::JointInfo :md5sum-) "52ccea37fd6730979bc7bbe9557a5b06")
(setf (get qt_nuitrack_app::JointInfo :datatype-) "qt_nuitrack_app/JointInfo")
(setf (get qt_nuitrack_app::JointInfo :definition-)
      "#std_msgs/Header header
uint8 type
float32 confidence
float32[] real
float32[] projection
float32[] orientation

")



(provide :qt_nuitrack_app/JointInfo "52ccea37fd6730979bc7bbe9557a5b06")


