; Auto-generated. Do not edit!


(cl:in-package lane_follower-msg)


;//! \htmlinclude VeronicaStatusReport.msg.html

(cl:defclass <VeronicaStatusReport> (roslisp-msg-protocol:ros-message)
  ((IMU_STATUS
    :reader IMU_STATUS
    :initarg :IMU_STATUS
    :type cl:boolean
    :initform cl:nil)
   (CAMERA_STATUS
    :reader CAMERA_STATUS
    :initarg :CAMERA_STATUS
    :type cl:boolean
    :initform cl:nil)
   (GPS_STATUS
    :reader GPS_STATUS
    :initarg :GPS_STATUS
    :type cl:boolean
    :initform cl:nil)
   (LIDAR_STATUS
    :reader LIDAR_STATUS
    :initarg :LIDAR_STATUS
    :type cl:boolean
    :initform cl:nil)
   (MOTOR_DRIVER_STATUS
    :reader MOTOR_DRIVER_STATUS
    :initarg :MOTOR_DRIVER_STATUS
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass VeronicaStatusReport (<VeronicaStatusReport>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VeronicaStatusReport>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VeronicaStatusReport)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lane_follower-msg:<VeronicaStatusReport> is deprecated: use lane_follower-msg:VeronicaStatusReport instead.")))

(cl:ensure-generic-function 'IMU_STATUS-val :lambda-list '(m))
(cl:defmethod IMU_STATUS-val ((m <VeronicaStatusReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lane_follower-msg:IMU_STATUS-val is deprecated.  Use lane_follower-msg:IMU_STATUS instead.")
  (IMU_STATUS m))

(cl:ensure-generic-function 'CAMERA_STATUS-val :lambda-list '(m))
(cl:defmethod CAMERA_STATUS-val ((m <VeronicaStatusReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lane_follower-msg:CAMERA_STATUS-val is deprecated.  Use lane_follower-msg:CAMERA_STATUS instead.")
  (CAMERA_STATUS m))

(cl:ensure-generic-function 'GPS_STATUS-val :lambda-list '(m))
(cl:defmethod GPS_STATUS-val ((m <VeronicaStatusReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lane_follower-msg:GPS_STATUS-val is deprecated.  Use lane_follower-msg:GPS_STATUS instead.")
  (GPS_STATUS m))

(cl:ensure-generic-function 'LIDAR_STATUS-val :lambda-list '(m))
(cl:defmethod LIDAR_STATUS-val ((m <VeronicaStatusReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lane_follower-msg:LIDAR_STATUS-val is deprecated.  Use lane_follower-msg:LIDAR_STATUS instead.")
  (LIDAR_STATUS m))

(cl:ensure-generic-function 'MOTOR_DRIVER_STATUS-val :lambda-list '(m))
(cl:defmethod MOTOR_DRIVER_STATUS-val ((m <VeronicaStatusReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lane_follower-msg:MOTOR_DRIVER_STATUS-val is deprecated.  Use lane_follower-msg:MOTOR_DRIVER_STATUS instead.")
  (MOTOR_DRIVER_STATUS m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VeronicaStatusReport>) ostream)
  "Serializes a message object of type '<VeronicaStatusReport>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'IMU_STATUS) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'CAMERA_STATUS) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'GPS_STATUS) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'LIDAR_STATUS) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'MOTOR_DRIVER_STATUS) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VeronicaStatusReport>) istream)
  "Deserializes a message object of type '<VeronicaStatusReport>"
    (cl:setf (cl:slot-value msg 'IMU_STATUS) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'CAMERA_STATUS) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'GPS_STATUS) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'LIDAR_STATUS) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'MOTOR_DRIVER_STATUS) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VeronicaStatusReport>)))
  "Returns string type for a message object of type '<VeronicaStatusReport>"
  "lane_follower/VeronicaStatusReport")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VeronicaStatusReport)))
  "Returns string type for a message object of type 'VeronicaStatusReport"
  "lane_follower/VeronicaStatusReport")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VeronicaStatusReport>)))
  "Returns md5sum for a message object of type '<VeronicaStatusReport>"
  "91482b64151ad166efd0cf38c80d57a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VeronicaStatusReport)))
  "Returns md5sum for a message object of type 'VeronicaStatusReport"
  "91482b64151ad166efd0cf38c80d57a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VeronicaStatusReport>)))
  "Returns full string definition for message of type '<VeronicaStatusReport>"
  (cl:format cl:nil "bool IMU_STATUS~%bool CAMERA_STATUS~%bool GPS_STATUS~%bool LIDAR_STATUS~%bool MOTOR_DRIVER_STATUS~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VeronicaStatusReport)))
  "Returns full string definition for message of type 'VeronicaStatusReport"
  (cl:format cl:nil "bool IMU_STATUS~%bool CAMERA_STATUS~%bool GPS_STATUS~%bool LIDAR_STATUS~%bool MOTOR_DRIVER_STATUS~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VeronicaStatusReport>))
  (cl:+ 0
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VeronicaStatusReport>))
  "Converts a ROS message object to a list"
  (cl:list 'VeronicaStatusReport
    (cl:cons ':IMU_STATUS (IMU_STATUS msg))
    (cl:cons ':CAMERA_STATUS (CAMERA_STATUS msg))
    (cl:cons ':GPS_STATUS (GPS_STATUS msg))
    (cl:cons ':LIDAR_STATUS (LIDAR_STATUS msg))
    (cl:cons ':MOTOR_DRIVER_STATUS (MOTOR_DRIVER_STATUS msg))
))
