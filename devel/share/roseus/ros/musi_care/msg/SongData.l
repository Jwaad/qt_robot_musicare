;; Auto-generated. Do not edit!


(when (boundp 'musi_care::SongData)
  (if (not (find-package "MUSI_CARE"))
    (make-package "MUSI_CARE"))
  (shadow 'SongData (find-package "MUSI_CARE")))
(unless (find-package "MUSI_CARE::SONGDATA")
  (make-package "MUSI_CARE::SONGDATA"))

(in-package "ROS")
;;//! \htmlinclude SongData.msg.html


(defclass musi_care::SongData
  :super ros::object
  :slots (_track_title _track_elapsed_time _track_total_time ))

(defmethod musi_care::SongData
  (:init
   (&key
    ((:track_title __track_title) "")
    ((:track_elapsed_time __track_elapsed_time) 0.0)
    ((:track_total_time __track_total_time) 0.0)
    )
   (send-super :init)
   (setq _track_title (string __track_title))
   (setq _track_elapsed_time (float __track_elapsed_time))
   (setq _track_total_time (float __track_total_time))
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
  (:serialization-length
   ()
   (+
    ;; string _track_title
    4 (length _track_title)
    ;; float32 _track_elapsed_time
    4
    ;; float32 _track_total_time
    4
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
   ;;
   self)
  )

(setf (get musi_care::SongData :md5sum-) "82324e20ca3e42f8d6a8f9e9503524ba")
(setf (get musi_care::SongData :datatype-) "musi_care/SongData")
(setf (get musi_care::SongData :definition-)
      "string track_title
float32 track_elapsed_time
float32 track_total_time

")



(provide :musi_care/SongData "82324e20ca3e42f8d6a8f9e9503524ba")


