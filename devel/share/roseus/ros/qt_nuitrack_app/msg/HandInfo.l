;; Auto-generated. Do not edit!


(when (boundp 'qt_nuitrack_app::HandInfo)
  (if (not (find-package "QT_NUITRACK_APP"))
    (make-package "QT_NUITRACK_APP"))
  (shadow 'HandInfo (find-package "QT_NUITRACK_APP")))
(unless (find-package "QT_NUITRACK_APP::HANDINFO")
  (make-package "QT_NUITRACK_APP::HANDINFO"))

(in-package "ROS")
;;//! \htmlinclude HandInfo.msg.html


(defclass qt_nuitrack_app::HandInfo
  :super ros::object
  :slots (_id _right_projection _right_real _right_click _right_pressure _left_projection _left_real _left_click _left_pressure ))

(defmethod qt_nuitrack_app::HandInfo
  (:init
   (&key
    ((:id __id) 0)
    ((:right_projection __right_projection) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:right_real __right_real) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:right_click __right_click) nil)
    ((:right_pressure __right_pressure) 0)
    ((:left_projection __left_projection) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:left_real __left_real) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:left_click __left_click) nil)
    ((:left_pressure __left_pressure) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _right_projection __right_projection)
   (setq _right_real __right_real)
   (setq _right_click __right_click)
   (setq _right_pressure (round __right_pressure))
   (setq _left_projection __left_projection)
   (setq _left_real __left_real)
   (setq _left_click __left_click)
   (setq _left_pressure (round __left_pressure))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:right_projection
   (&optional __right_projection)
   (if __right_projection (setq _right_projection __right_projection)) _right_projection)
  (:right_real
   (&optional __right_real)
   (if __right_real (setq _right_real __right_real)) _right_real)
  (:right_click
   (&optional (__right_click :null))
   (if (not (eq __right_click :null)) (setq _right_click __right_click)) _right_click)
  (:right_pressure
   (&optional __right_pressure)
   (if __right_pressure (setq _right_pressure __right_pressure)) _right_pressure)
  (:left_projection
   (&optional __left_projection)
   (if __left_projection (setq _left_projection __left_projection)) _left_projection)
  (:left_real
   (&optional __left_real)
   (if __left_real (setq _left_real __left_real)) _left_real)
  (:left_click
   (&optional (__left_click :null))
   (if (not (eq __left_click :null)) (setq _left_click __left_click)) _left_click)
  (:left_pressure
   (&optional __left_pressure)
   (if __left_pressure (setq _left_pressure __left_pressure)) _left_pressure)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; float32[] _right_projection
    (* 4    (length _right_projection)) 4
    ;; float32[] _right_real
    (* 4    (length _right_real)) 4
    ;; bool _right_click
    1
    ;; int32 _right_pressure
    4
    ;; float32[] _left_projection
    (* 4    (length _left_projection)) 4
    ;; float32[] _left_real
    (* 4    (length _left_real)) 4
    ;; bool _left_click
    1
    ;; int32 _left_pressure
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; float32[] _right_projection
     (write-long (length _right_projection) s)
     (dotimes (i (length _right_projection))
       (sys::poke (elt _right_projection i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _right_real
     (write-long (length _right_real) s)
     (dotimes (i (length _right_real))
       (sys::poke (elt _right_real i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _right_click
       (if _right_click (write-byte -1 s) (write-byte 0 s))
     ;; int32 _right_pressure
       (write-long _right_pressure s)
     ;; float32[] _left_projection
     (write-long (length _left_projection) s)
     (dotimes (i (length _left_projection))
       (sys::poke (elt _left_projection i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _left_real
     (write-long (length _left_real) s)
     (dotimes (i (length _left_real))
       (sys::poke (elt _left_real i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _left_click
       (if _left_click (write-byte -1 s) (write-byte 0 s))
     ;; int32 _left_pressure
       (write-long _left_pressure s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _right_projection
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _right_projection (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _right_projection i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _right_real
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _right_real (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _right_real i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _right_click
     (setq _right_click (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _right_pressure
     (setq _right_pressure (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _left_projection
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _left_projection (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _left_projection i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _left_real
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _left_real (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _left_real i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _left_click
     (setq _left_click (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _left_pressure
     (setq _left_pressure (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get qt_nuitrack_app::HandInfo :md5sum-) "ebda9d77d8f24f2a507d651777ac1949")
(setf (get qt_nuitrack_app::HandInfo :datatype-) "qt_nuitrack_app/HandInfo")
(setf (get qt_nuitrack_app::HandInfo :definition-)
      "#std_msgs/Header header
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



(provide :qt_nuitrack_app/HandInfo "ebda9d77d8f24f2a507d651777ac1949")


