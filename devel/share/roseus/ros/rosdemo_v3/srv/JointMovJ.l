;; Auto-generated. Do not edit!


(when (boundp 'rosdemo_v3::JointMovJ)
  (if (not (find-package "ROSDEMO_V3"))
    (make-package "ROSDEMO_V3"))
  (shadow 'JointMovJ (find-package "ROSDEMO_V3")))
(unless (find-package "ROSDEMO_V3::JOINTMOVJ")
  (make-package "ROSDEMO_V3::JOINTMOVJ"))
(unless (find-package "ROSDEMO_V3::JOINTMOVJREQUEST")
  (make-package "ROSDEMO_V3::JOINTMOVJREQUEST"))
(unless (find-package "ROSDEMO_V3::JOINTMOVJRESPONSE")
  (make-package "ROSDEMO_V3::JOINTMOVJRESPONSE"))

(in-package "ROS")





(defclass rosdemo_v3::JointMovJRequest
  :super ros::object
  :slots (_j1 _j2 _j3 _j4 _j5 _j6 _paramValue ))

(defmethod rosdemo_v3::JointMovJRequest
  (:init
   (&key
    ((:j1 __j1) 0.0)
    ((:j2 __j2) 0.0)
    ((:j3 __j3) 0.0)
    ((:j4 __j4) 0.0)
    ((:j5 __j5) 0.0)
    ((:j6 __j6) 0.0)
    ((:paramValue __paramValue) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _j1 (float __j1))
   (setq _j2 (float __j2))
   (setq _j3 (float __j3))
   (setq _j4 (float __j4))
   (setq _j5 (float __j5))
   (setq _j6 (float __j6))
   (setq _paramValue __paramValue)
   self)
  (:j1
   (&optional __j1)
   (if __j1 (setq _j1 __j1)) _j1)
  (:j2
   (&optional __j2)
   (if __j2 (setq _j2 __j2)) _j2)
  (:j3
   (&optional __j3)
   (if __j3 (setq _j3 __j3)) _j3)
  (:j4
   (&optional __j4)
   (if __j4 (setq _j4 __j4)) _j4)
  (:j5
   (&optional __j5)
   (if __j5 (setq _j5 __j5)) _j5)
  (:j6
   (&optional __j6)
   (if __j6 (setq _j6 __j6)) _j6)
  (:paramValue
   (&optional __paramValue)
   (if __paramValue (setq _paramValue __paramValue)) _paramValue)
  (:serialization-length
   ()
   (+
    ;; float64 _j1
    8
    ;; float64 _j2
    8
    ;; float64 _j3
    8
    ;; float64 _j4
    8
    ;; float64 _j5
    8
    ;; float64 _j6
    8
    ;; string[] _paramValue
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _paramValue)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _j1
       (sys::poke _j1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _j2
       (sys::poke _j2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _j3
       (sys::poke _j3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _j4
       (sys::poke _j4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _j5
       (sys::poke _j5 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _j6
       (sys::poke _j6 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string[] _paramValue
     (write-long (length _paramValue) s)
     (dolist (elem _paramValue)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _j1
     (setq _j1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _j2
     (setq _j2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _j3
     (setq _j3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _j4
     (setq _j4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _j5
     (setq _j5 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _j6
     (setq _j6 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string[] _paramValue
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _paramValue (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _paramValue i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass rosdemo_v3::JointMovJResponse
  :super ros::object
  :slots (_res ))

(defmethod rosdemo_v3::JointMovJResponse
  (:init
   (&key
    ((:res __res) 0)
    )
   (send-super :init)
   (setq _res (round __res))
   self)
  (:res
   (&optional __res)
   (if __res (setq _res __res)) _res)
  (:serialization-length
   ()
   (+
    ;; int32 _res
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _res
       (write-long _res s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _res
     (setq _res (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass rosdemo_v3::JointMovJ
  :super ros::object
  :slots ())

(setf (get rosdemo_v3::JointMovJ :md5sum-) "1017b68b63f136fff53e96a209a4d563")
(setf (get rosdemo_v3::JointMovJ :datatype-) "rosdemo_v3/JointMovJ")
(setf (get rosdemo_v3::JointMovJ :request) rosdemo_v3::JointMovJRequest)
(setf (get rosdemo_v3::JointMovJ :response) rosdemo_v3::JointMovJResponse)

(defmethod rosdemo_v3::JointMovJRequest
  (:response () (instance rosdemo_v3::JointMovJResponse :init)))

(setf (get rosdemo_v3::JointMovJRequest :md5sum-) "1017b68b63f136fff53e96a209a4d563")
(setf (get rosdemo_v3::JointMovJRequest :datatype-) "rosdemo_v3/JointMovJRequest")
(setf (get rosdemo_v3::JointMovJRequest :definition-)
      "float64 j1
float64 j2
float64 j3
float64 j4
float64 j5
float64 j6
string[] paramValue
---
int32 res
")

(setf (get rosdemo_v3::JointMovJResponse :md5sum-) "1017b68b63f136fff53e96a209a4d563")
(setf (get rosdemo_v3::JointMovJResponse :datatype-) "rosdemo_v3/JointMovJResponse")
(setf (get rosdemo_v3::JointMovJResponse :definition-)
      "float64 j1
float64 j2
float64 j3
float64 j4
float64 j5
float64 j6
string[] paramValue
---
int32 res
")



(provide :rosdemo_v3/JointMovJ "1017b68b63f136fff53e96a209a4d563")


