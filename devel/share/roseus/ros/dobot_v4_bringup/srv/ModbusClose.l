;; Auto-generated. Do not edit!


(when (boundp 'dobot_v4_bringup::ModbusClose)
  (if (not (find-package "DOBOT_V4_BRINGUP"))
    (make-package "DOBOT_V4_BRINGUP"))
  (shadow 'ModbusClose (find-package "DOBOT_V4_BRINGUP")))
(unless (find-package "DOBOT_V4_BRINGUP::MODBUSCLOSE")
  (make-package "DOBOT_V4_BRINGUP::MODBUSCLOSE"))
(unless (find-package "DOBOT_V4_BRINGUP::MODBUSCLOSEREQUEST")
  (make-package "DOBOT_V4_BRINGUP::MODBUSCLOSEREQUEST"))
(unless (find-package "DOBOT_V4_BRINGUP::MODBUSCLOSERESPONSE")
  (make-package "DOBOT_V4_BRINGUP::MODBUSCLOSERESPONSE"))

(in-package "ROS")





(defclass dobot_v4_bringup::ModbusCloseRequest
  :super ros::object
  :slots (_index ))

(defmethod dobot_v4_bringup::ModbusCloseRequest
  (:init
   (&key
    ((:index __index) 0)
    )
   (send-super :init)
   (setq _index (round __index))
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:serialization-length
   ()
   (+
    ;; int32 _index
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _index
       (write-long _index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _index
     (setq _index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dobot_v4_bringup::ModbusCloseResponse
  :super ros::object
  :slots (_res ))

(defmethod dobot_v4_bringup::ModbusCloseResponse
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

(defclass dobot_v4_bringup::ModbusClose
  :super ros::object
  :slots ())

(setf (get dobot_v4_bringup::ModbusClose :md5sum-) "6d753c0adbdd6c06a6eb5c36aec96b0c")
(setf (get dobot_v4_bringup::ModbusClose :datatype-) "dobot_v4_bringup/ModbusClose")
(setf (get dobot_v4_bringup::ModbusClose :request) dobot_v4_bringup::ModbusCloseRequest)
(setf (get dobot_v4_bringup::ModbusClose :response) dobot_v4_bringup::ModbusCloseResponse)

(defmethod dobot_v4_bringup::ModbusCloseRequest
  (:response () (instance dobot_v4_bringup::ModbusCloseResponse :init)))

(setf (get dobot_v4_bringup::ModbusCloseRequest :md5sum-) "6d753c0adbdd6c06a6eb5c36aec96b0c")
(setf (get dobot_v4_bringup::ModbusCloseRequest :datatype-) "dobot_v4_bringup/ModbusCloseRequest")
(setf (get dobot_v4_bringup::ModbusCloseRequest :definition-)
      "int32 index
---
int32 res
")

(setf (get dobot_v4_bringup::ModbusCloseResponse :md5sum-) "6d753c0adbdd6c06a6eb5c36aec96b0c")
(setf (get dobot_v4_bringup::ModbusCloseResponse :datatype-) "dobot_v4_bringup/ModbusCloseResponse")
(setf (get dobot_v4_bringup::ModbusCloseResponse :definition-)
      "int32 index
---
int32 res
")



(provide :dobot_v4_bringup/ModbusClose "6d753c0adbdd6c06a6eb5c36aec96b0c")


