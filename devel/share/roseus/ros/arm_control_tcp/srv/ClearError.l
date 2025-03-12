;; Auto-generated. Do not edit!


(when (boundp 'arm_control_tcp::ClearError)
  (if (not (find-package "ARM_CONTROL_TCP"))
    (make-package "ARM_CONTROL_TCP"))
  (shadow 'ClearError (find-package "ARM_CONTROL_TCP")))
(unless (find-package "ARM_CONTROL_TCP::CLEARERROR")
  (make-package "ARM_CONTROL_TCP::CLEARERROR"))
(unless (find-package "ARM_CONTROL_TCP::CLEARERRORREQUEST")
  (make-package "ARM_CONTROL_TCP::CLEARERRORREQUEST"))
(unless (find-package "ARM_CONTROL_TCP::CLEARERRORRESPONSE")
  (make-package "ARM_CONTROL_TCP::CLEARERRORRESPONSE"))

(in-package "ROS")





(defclass arm_control_tcp::ClearErrorRequest
  :super ros::object
  :slots ())

(defmethod arm_control_tcp::ClearErrorRequest
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

(defclass arm_control_tcp::ClearErrorResponse
  :super ros::object
  :slots (_res ))

(defmethod arm_control_tcp::ClearErrorResponse
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

(defclass arm_control_tcp::ClearError
  :super ros::object
  :slots ())

(setf (get arm_control_tcp::ClearError :md5sum-) "ca16cfbd5443ad97f6cc7ffd6bb67292")
(setf (get arm_control_tcp::ClearError :datatype-) "arm_control_tcp/ClearError")
(setf (get arm_control_tcp::ClearError :request) arm_control_tcp::ClearErrorRequest)
(setf (get arm_control_tcp::ClearError :response) arm_control_tcp::ClearErrorResponse)

(defmethod arm_control_tcp::ClearErrorRequest
  (:response () (instance arm_control_tcp::ClearErrorResponse :init)))

(setf (get arm_control_tcp::ClearErrorRequest :md5sum-) "ca16cfbd5443ad97f6cc7ffd6bb67292")
(setf (get arm_control_tcp::ClearErrorRequest :datatype-) "arm_control_tcp/ClearErrorRequest")
(setf (get arm_control_tcp::ClearErrorRequest :definition-)
      "---
int32 res
")

(setf (get arm_control_tcp::ClearErrorResponse :md5sum-) "ca16cfbd5443ad97f6cc7ffd6bb67292")
(setf (get arm_control_tcp::ClearErrorResponse :datatype-) "arm_control_tcp/ClearErrorResponse")
(setf (get arm_control_tcp::ClearErrorResponse :definition-)
      "---
int32 res
")



(provide :arm_control_tcp/ClearError "ca16cfbd5443ad97f6cc7ffd6bb67292")


