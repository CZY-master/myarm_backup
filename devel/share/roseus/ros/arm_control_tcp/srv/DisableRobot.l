;; Auto-generated. Do not edit!


(when (boundp 'arm_control_tcp::DisableRobot)
  (if (not (find-package "ARM_CONTROL_TCP"))
    (make-package "ARM_CONTROL_TCP"))
  (shadow 'DisableRobot (find-package "ARM_CONTROL_TCP")))
(unless (find-package "ARM_CONTROL_TCP::DISABLEROBOT")
  (make-package "ARM_CONTROL_TCP::DISABLEROBOT"))
(unless (find-package "ARM_CONTROL_TCP::DISABLEROBOTREQUEST")
  (make-package "ARM_CONTROL_TCP::DISABLEROBOTREQUEST"))
(unless (find-package "ARM_CONTROL_TCP::DISABLEROBOTRESPONSE")
  (make-package "ARM_CONTROL_TCP::DISABLEROBOTRESPONSE"))

(in-package "ROS")





(defclass arm_control_tcp::DisableRobotRequest
  :super ros::object
  :slots ())

(defmethod arm_control_tcp::DisableRobotRequest
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

(defclass arm_control_tcp::DisableRobotResponse
  :super ros::object
  :slots (_res ))

(defmethod arm_control_tcp::DisableRobotResponse
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

(defclass arm_control_tcp::DisableRobot
  :super ros::object
  :slots ())

(setf (get arm_control_tcp::DisableRobot :md5sum-) "ca16cfbd5443ad97f6cc7ffd6bb67292")
(setf (get arm_control_tcp::DisableRobot :datatype-) "arm_control_tcp/DisableRobot")
(setf (get arm_control_tcp::DisableRobot :request) arm_control_tcp::DisableRobotRequest)
(setf (get arm_control_tcp::DisableRobot :response) arm_control_tcp::DisableRobotResponse)

(defmethod arm_control_tcp::DisableRobotRequest
  (:response () (instance arm_control_tcp::DisableRobotResponse :init)))

(setf (get arm_control_tcp::DisableRobotRequest :md5sum-) "ca16cfbd5443ad97f6cc7ffd6bb67292")
(setf (get arm_control_tcp::DisableRobotRequest :datatype-) "arm_control_tcp/DisableRobotRequest")
(setf (get arm_control_tcp::DisableRobotRequest :definition-)
      "---
int32 res
")

(setf (get arm_control_tcp::DisableRobotResponse :md5sum-) "ca16cfbd5443ad97f6cc7ffd6bb67292")
(setf (get arm_control_tcp::DisableRobotResponse :datatype-) "arm_control_tcp/DisableRobotResponse")
(setf (get arm_control_tcp::DisableRobotResponse :definition-)
      "---
int32 res
")



(provide :arm_control_tcp/DisableRobot "ca16cfbd5443ad97f6cc7ffd6bb67292")


