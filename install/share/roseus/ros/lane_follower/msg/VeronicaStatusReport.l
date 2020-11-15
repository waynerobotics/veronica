;; Auto-generated. Do not edit!


(when (boundp 'lane_follower::VeronicaStatusReport)
  (if (not (find-package "LANE_FOLLOWER"))
    (make-package "LANE_FOLLOWER"))
  (shadow 'VeronicaStatusReport (find-package "LANE_FOLLOWER")))
(unless (find-package "LANE_FOLLOWER::VERONICASTATUSREPORT")
  (make-package "LANE_FOLLOWER::VERONICASTATUSREPORT"))

(in-package "ROS")
;;//! \htmlinclude VeronicaStatusReport.msg.html


(defclass lane_follower::VeronicaStatusReport
  :super ros::object
  :slots (_IMU_STATUS _CAMERA_STATUS _GPS_STATUS _LIDAR_STATUS _MOTOR_DRIVER_STATUS ))

(defmethod lane_follower::VeronicaStatusReport
  (:init
   (&key
    ((:IMU_STATUS __IMU_STATUS) nil)
    ((:CAMERA_STATUS __CAMERA_STATUS) nil)
    ((:GPS_STATUS __GPS_STATUS) nil)
    ((:LIDAR_STATUS __LIDAR_STATUS) nil)
    ((:MOTOR_DRIVER_STATUS __MOTOR_DRIVER_STATUS) nil)
    )
   (send-super :init)
   (setq _IMU_STATUS __IMU_STATUS)
   (setq _CAMERA_STATUS __CAMERA_STATUS)
   (setq _GPS_STATUS __GPS_STATUS)
   (setq _LIDAR_STATUS __LIDAR_STATUS)
   (setq _MOTOR_DRIVER_STATUS __MOTOR_DRIVER_STATUS)
   self)
  (:IMU_STATUS
   (&optional __IMU_STATUS)
   (if __IMU_STATUS (setq _IMU_STATUS __IMU_STATUS)) _IMU_STATUS)
  (:CAMERA_STATUS
   (&optional __CAMERA_STATUS)
   (if __CAMERA_STATUS (setq _CAMERA_STATUS __CAMERA_STATUS)) _CAMERA_STATUS)
  (:GPS_STATUS
   (&optional __GPS_STATUS)
   (if __GPS_STATUS (setq _GPS_STATUS __GPS_STATUS)) _GPS_STATUS)
  (:LIDAR_STATUS
   (&optional __LIDAR_STATUS)
   (if __LIDAR_STATUS (setq _LIDAR_STATUS __LIDAR_STATUS)) _LIDAR_STATUS)
  (:MOTOR_DRIVER_STATUS
   (&optional __MOTOR_DRIVER_STATUS)
   (if __MOTOR_DRIVER_STATUS (setq _MOTOR_DRIVER_STATUS __MOTOR_DRIVER_STATUS)) _MOTOR_DRIVER_STATUS)
  (:serialization-length
   ()
   (+
    ;; bool _IMU_STATUS
    1
    ;; bool _CAMERA_STATUS
    1
    ;; bool _GPS_STATUS
    1
    ;; bool _LIDAR_STATUS
    1
    ;; bool _MOTOR_DRIVER_STATUS
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _IMU_STATUS
       (if _IMU_STATUS (write-byte -1 s) (write-byte 0 s))
     ;; bool _CAMERA_STATUS
       (if _CAMERA_STATUS (write-byte -1 s) (write-byte 0 s))
     ;; bool _GPS_STATUS
       (if _GPS_STATUS (write-byte -1 s) (write-byte 0 s))
     ;; bool _LIDAR_STATUS
       (if _LIDAR_STATUS (write-byte -1 s) (write-byte 0 s))
     ;; bool _MOTOR_DRIVER_STATUS
       (if _MOTOR_DRIVER_STATUS (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _IMU_STATUS
     (setq _IMU_STATUS (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _CAMERA_STATUS
     (setq _CAMERA_STATUS (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _GPS_STATUS
     (setq _GPS_STATUS (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _LIDAR_STATUS
     (setq _LIDAR_STATUS (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _MOTOR_DRIVER_STATUS
     (setq _MOTOR_DRIVER_STATUS (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get lane_follower::VeronicaStatusReport :md5sum-) "91482b64151ad166efd0cf38c80d57a6")
(setf (get lane_follower::VeronicaStatusReport :datatype-) "lane_follower/VeronicaStatusReport")
(setf (get lane_follower::VeronicaStatusReport :definition-)
      "bool IMU_STATUS
bool CAMERA_STATUS
bool GPS_STATUS
bool LIDAR_STATUS
bool MOTOR_DRIVER_STATUS


")



(provide :lane_follower/VeronicaStatusReport "91482b64151ad166efd0cf38c80d57a6")


