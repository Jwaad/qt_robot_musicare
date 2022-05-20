;; Auto-generated. Do not edit!


(when (boundp 'musi_care::sound_player_srv)
  (if (not (find-package "MUSI_CARE"))
    (make-package "MUSI_CARE"))
  (shadow 'sound_player_srv (find-package "MUSI_CARE")))
(unless (find-package "MUSI_CARE::SOUND_PLAYER_SRV")
  (make-package "MUSI_CARE::SOUND_PLAYER_SRV"))
(unless (find-package "MUSI_CARE::SOUND_PLAYER_SRVREQUEST")
  (make-package "MUSI_CARE::SOUND_PLAYER_SRVREQUEST"))
(unless (find-package "MUSI_CARE::SOUND_PLAYER_SRVRESPONSE")
  (make-package "MUSI_CARE::SOUND_PLAYER_SRVRESPONSE"))

(in-package "ROS")





(defclass musi_care::sound_player_srvRequest
  :super ros::object
  :slots (_operation _operation_data_1 _operation_data_2 ))

(defmethod musi_care::sound_player_srvRequest
  (:init
   (&key
    ((:operation __operation) "")
    ((:operation_data_1 __operation_data_1) "")
    ((:operation_data_2 __operation_data_2) 0.0)
    )
   (send-super :init)
   (setq _operation (string __operation))
   (setq _operation_data_1 (string __operation_data_1))
   (setq _operation_data_2 (float __operation_data_2))
   self)
  (:operation
   (&optional __operation)
   (if __operation (setq _operation __operation)) _operation)
  (:operation_data_1
   (&optional __operation_data_1)
   (if __operation_data_1 (setq _operation_data_1 __operation_data_1)) _operation_data_1)
  (:operation_data_2
   (&optional __operation_data_2)
   (if __operation_data_2 (setq _operation_data_2 __operation_data_2)) _operation_data_2)
  (:serialization-length
   ()
   (+
    ;; string _operation
    4 (length _operation)
    ;; string _operation_data_1
    4 (length _operation_data_1)
    ;; float32 _operation_data_2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _operation
       (write-long (length _operation) s) (princ _operation s)
     ;; string _operation_data_1
       (write-long (length _operation_data_1) s) (princ _operation_data_1 s)
     ;; float32 _operation_data_2
       (sys::poke _operation_data_2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _operation
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _operation (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _operation_data_1
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _operation_data_1 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _operation_data_2
     (setq _operation_data_2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass musi_care::sound_player_srvResponse
  :super ros::object
  :slots (_track_title _track_elapsed_time _track_total_time _status ))

(defmethod musi_care::sound_player_srvResponse
  (:init
   (&key
    ((:track_title __track_title) "")
    ((:track_elapsed_time __track_elapsed_time) 0.0)
    ((:track_total_time __track_total_time) 0.0)
    ((:status __status) nil)
    )
   (send-super :init)
   (setq _track_title (string __track_title))
   (setq _track_elapsed_time (float __track_elapsed_time))
   (setq _track_total_time (float __track_total_time))
   (setq _status __status)
   self)
  (:track_title
   (&optional __track_title)
   (if __track_title (setq _track_title __track_title)) _track_title)
  (:track_elapsed_time
   (&optional __track_elapsed_time)
   (if __track_elapsed_time (setq _track_elapsed_time __track_elapsed_time)) _track_elapsed_time)
  (:track_total_time
   (&optional __track_total_time)
   (if __track_total_time (setq _track_total_time __track_total_time)) _track_total_time)
  (:status
   (&optional (__status :null))
   (if (not (eq __status :null)) (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; string _track_title
    4 (length _track_title)
    ;; float32 _track_elapsed_time
    4
    ;; float32 _track_total_time
    4
    ;; bool _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _track_title
       (write-long (length _track_title) s) (princ _track_title s)
     ;; float32 _track_elapsed_time
       (sys::poke _track_elapsed_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _track_total_time
       (sys::poke _track_total_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _track_title
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _track_title (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _track_elapsed_time
     (setq _track_elapsed_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _track_total_time
     (setq _track_total_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass musi_care::sound_player_srv
  :super ros::object
  :slots ())

(setf (get musi_care::sound_player_srv :md5sum-) "2a3bef821864039a35e20805c207d7a8")
(setf (get musi_care::sound_player_srv :datatype-) "musi_care/sound_player_srv")
(setf (get musi_care::sound_player_srv :request) musi_care::sound_player_srvRequest)
(setf (get musi_care::sound_player_srv :response) musi_care::sound_player_srvResponse)

(defmethod musi_care::sound_player_srvRequest
  (:response () (instance musi_care::sound_player_srvResponse :init)))

(setf (get musi_care::sound_player_srvRequest :md5sum-) "2a3bef821864039a35e20805c207d7a8")
(setf (get musi_care::sound_player_srvRequest :datatype-) "musi_care/sound_player_srvRequest")
(setf (get musi_care::sound_player_srvRequest :definition-)
      "string operation
string operation_data_1
float32 operation_data_2
---
string track_title
float32 track_elapsed_time
float32 track_total_time
bool status

")

(setf (get musi_care::sound_player_srvResponse :md5sum-) "2a3bef821864039a35e20805c207d7a8")
(setf (get musi_care::sound_player_srvResponse :datatype-) "musi_care/sound_player_srvResponse")
(setf (get musi_care::sound_player_srvResponse :definition-)
      "string operation
string operation_data_1
float32 operation_data_2
---
string track_title
float32 track_elapsed_time
float32 track_total_time
bool status

")



(provide :musi_care/sound_player_srv "2a3bef821864039a35e20805c207d7a8")


