;; Auto-generated. Do not edit!


(when (boundp 'arm_control_tcp::EnableRobot)
  (if (not (find-package "ARM_CONTROL_TCP"))
    (make-package "ARM_CONTROL_TCP"))
  (shadow 'EnableRobot (find-package "ARM_CONTROL_TCP")))
(unless (find-package "ARM_CONTROL_TCP::ENABLEROBOT")
  (make-package "ARM_CONTROL_TCP::ENABLEROBOT"))
(unless (find-package "ARM_CONTROL_TCP::ENABLEROBOTREQUEST")
  (make-package "ARM_CONTROL_TCP::ENABLEROBOTREQUEST"))
(unless (find-package "ARM_CONTROL_TCP::ENABLEROBOTRESPONSE")
  (make-package "ARM_CONTROL_TCP::ENABLEROBOTRESPONSE"))

(in-package "ROS")





(defclass arm_control_tcp::EnableRobotRequest
  :super ros::object
  :slots ())

(defmethod arm_control_tcp::EnableRobotRequest
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

(defclass arm_control_tcp::EnableRobotResponse
  :super ros::object
  :slots (_res ))

(defmethod arm_control_tcp::EnableRobotResponse
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

(defclass arm_control_tcp::EnableRobot
  :super ros::object
  :slots ())

(setf (get arm_control_tcp::EnableRobot :md5sum-) "ca16cfbd5443ad97f6cc7ffd6bb67292")
(setf (get arm_control_tcp::EnableRobot :datatype-) "arm_control_tcp/EnableRobot")
(setf (get arm_control_tcp::EnableRobot :request) arm_control_tcp::EnableRobotRequest)
(setf (get arm_control_tcp::EnableRobot :response) arm_control_tcp::EnableRobotResponse)

(defmethod arm_control_tcp::EnableRobotRequest
  (:response () (instance arm_control_tcp::EnableRobotResponse :init)))

(setf (get arm_control_tcp::EnableRobotRequest :md5sum-) "ca16cfbd5443ad97f6cc7ffd6bb67292")
(setf (get arm_control_tcp::EnableRobotRequest :datatype-) "arm_control_tcp/EnableRobotRequest")
(setf (get arm_control_tcp::EnableRobotRequest :definition-)
      "---
int32 res
")

(setf (get arm_control_tcp::EnableRobotResponse :md5sum-) "ca16cfbd5443ad97f6cc7ffd6bb67292")
(setf (get arm_control_tcp::EnableRobotResponse :datatype-) "arm_control_tcp/EnableRobotResponse")
(setf (get arm_control_tcp::EnableRobotResponse :definition-)
      "---
int32 res
")



(provide :arm_control_tcp/EnableRobot "ca16cfbd5443ad97f6cc7ffd6bb67292")


