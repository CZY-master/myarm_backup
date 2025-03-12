;; Auto-generated. Do not edit!


(when (boundp 'arm_control_tcp::GetErrorID)
  (if (not (find-package "ARM_CONTROL_TCP"))
    (make-package "ARM_CONTROL_TCP"))
  (shadow 'GetErrorID (find-package "ARM_CONTROL_TCP")))
(unless (find-package "ARM_CONTROL_TCP::GETERRORID")
  (make-package "ARM_CONTROL_TCP::GETERRORID"))
(unless (find-package "ARM_CONTROL_TCP::GETERRORIDREQUEST")
  (make-package "ARM_CONTROL_TCP::GETERRORIDREQUEST"))
(unless (find-package "ARM_CONTROL_TCP::GETERRORIDRESPONSE")
  (make-package "ARM_CONTROL_TCP::GETERRORIDRESPONSE"))

(in-package "ROS")





(defclass arm_control_tcp::GetErrorIDRequest
  :super ros::object
  :slots ())

(defmethod arm_control_tcp::GetErrorIDRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass arm_control_tcp::GetErrorIDResponse
  :super ros::object
  :slots (_error_id _res ))

(defmethod arm_control_tcp::GetErrorIDResponse
  (:init
   (&key
    ((:error_id __error_id) (make-array 0 :initial-element 0 :element-type :integer))
    ((:res __res) 0)
    )
   (send-super :init)
   (setq _error_id __error_id)
   (setq _res (round __res))
   self)
  (:error_id
   (&optional __error_id)
   (if __error_id (setq _error_id __error_id)) _error_id)
  (:res
   (&optional __res)
   (if __res (setq _res __res)) _res)
  (:serialization-length
   ()
   (+
    ;; int32[] _error_id
    (* 4    (length _error_id)) 4
    ;; int32 _res
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _error_id
     (write-long (length _error_id) s)
     (dotimes (i (length _error_id))
       (write-long (elt _error_id i) s)
       )
     ;; int32 _res
       (write-long _res s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _error_id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _error_id (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _error_id i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32 _res
     (setq _res (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass arm_control_tcp::GetErrorID
  :super ros::object
  :slots ())

(setf (get arm_control_tcp::GetErrorID :md5sum-) "15273c96ecb6b320e7c1e60e17abe5ab")
(setf (get arm_control_tcp::GetErrorID :datatype-) "arm_control_tcp/GetErrorID")
(setf (get arm_control_tcp::GetErrorID :request) arm_control_tcp::GetErrorIDRequest)
(setf (get arm_control_tcp::GetErrorID :response) arm_control_tcp::GetErrorIDResponse)

(defmethod arm_control_tcp::GetErrorIDRequest
  (:response () (instance arm_control_tcp::GetErrorIDResponse :init)))

(setf (get arm_control_tcp::GetErrorIDRequest :md5sum-) "15273c96ecb6b320e7c1e60e17abe5ab")
(setf (get arm_control_tcp::GetErrorIDRequest :datatype-) "arm_control_tcp/GetErrorIDRequest")
(setf (get arm_control_tcp::GetErrorIDRequest :definition-)
      "---
int32[] error_id
int32 res
")

(setf (get arm_control_tcp::GetErrorIDResponse :md5sum-) "15273c96ecb6b320e7c1e60e17abe5ab")
(setf (get arm_control_tcp::GetErrorIDResponse :datatype-) "arm_control_tcp/GetErrorIDResponse")
(setf (get arm_control_tcp::GetErrorIDResponse :definition-)
      "---
int32[] error_id
int32 res
")



(provide :arm_control_tcp/GetErrorID "15273c96ecb6b320e7c1e60e17abe5ab")


