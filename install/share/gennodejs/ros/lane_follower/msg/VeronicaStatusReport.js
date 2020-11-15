// Auto-generated. Do not edit!

// (in-package lane_follower.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class VeronicaStatusReport {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.IMU_STATUS = null;
      this.CAMERA_STATUS = null;
      this.GPS_STATUS = null;
      this.LIDAR_STATUS = null;
      this.MOTOR_DRIVER_STATUS = null;
    }
    else {
      if (initObj.hasOwnProperty('IMU_STATUS')) {
        this.IMU_STATUS = initObj.IMU_STATUS
      }
      else {
        this.IMU_STATUS = false;
      }
      if (initObj.hasOwnProperty('CAMERA_STATUS')) {
        this.CAMERA_STATUS = initObj.CAMERA_STATUS
      }
      else {
        this.CAMERA_STATUS = false;
      }
      if (initObj.hasOwnProperty('GPS_STATUS')) {
        this.GPS_STATUS = initObj.GPS_STATUS
      }
      else {
        this.GPS_STATUS = false;
      }
      if (initObj.hasOwnProperty('LIDAR_STATUS')) {
        this.LIDAR_STATUS = initObj.LIDAR_STATUS
      }
      else {
        this.LIDAR_STATUS = false;
      }
      if (initObj.hasOwnProperty('MOTOR_DRIVER_STATUS')) {
        this.MOTOR_DRIVER_STATUS = initObj.MOTOR_DRIVER_STATUS
      }
      else {
        this.MOTOR_DRIVER_STATUS = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VeronicaStatusReport
    // Serialize message field [IMU_STATUS]
    bufferOffset = _serializer.bool(obj.IMU_STATUS, buffer, bufferOffset);
    // Serialize message field [CAMERA_STATUS]
    bufferOffset = _serializer.bool(obj.CAMERA_STATUS, buffer, bufferOffset);
    // Serialize message field [GPS_STATUS]
    bufferOffset = _serializer.bool(obj.GPS_STATUS, buffer, bufferOffset);
    // Serialize message field [LIDAR_STATUS]
    bufferOffset = _serializer.bool(obj.LIDAR_STATUS, buffer, bufferOffset);
    // Serialize message field [MOTOR_DRIVER_STATUS]
    bufferOffset = _serializer.bool(obj.MOTOR_DRIVER_STATUS, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VeronicaStatusReport
    let len;
    let data = new VeronicaStatusReport(null);
    // Deserialize message field [IMU_STATUS]
    data.IMU_STATUS = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [CAMERA_STATUS]
    data.CAMERA_STATUS = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [GPS_STATUS]
    data.GPS_STATUS = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [LIDAR_STATUS]
    data.LIDAR_STATUS = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [MOTOR_DRIVER_STATUS]
    data.MOTOR_DRIVER_STATUS = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lane_follower/VeronicaStatusReport';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '91482b64151ad166efd0cf38c80d57a6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool IMU_STATUS
    bool CAMERA_STATUS
    bool GPS_STATUS
    bool LIDAR_STATUS
    bool MOTOR_DRIVER_STATUS
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VeronicaStatusReport(null);
    if (msg.IMU_STATUS !== undefined) {
      resolved.IMU_STATUS = msg.IMU_STATUS;
    }
    else {
      resolved.IMU_STATUS = false
    }

    if (msg.CAMERA_STATUS !== undefined) {
      resolved.CAMERA_STATUS = msg.CAMERA_STATUS;
    }
    else {
      resolved.CAMERA_STATUS = false
    }

    if (msg.GPS_STATUS !== undefined) {
      resolved.GPS_STATUS = msg.GPS_STATUS;
    }
    else {
      resolved.GPS_STATUS = false
    }

    if (msg.LIDAR_STATUS !== undefined) {
      resolved.LIDAR_STATUS = msg.LIDAR_STATUS;
    }
    else {
      resolved.LIDAR_STATUS = false
    }

    if (msg.MOTOR_DRIVER_STATUS !== undefined) {
      resolved.MOTOR_DRIVER_STATUS = msg.MOTOR_DRIVER_STATUS;
    }
    else {
      resolved.MOTOR_DRIVER_STATUS = false
    }

    return resolved;
    }
};

module.exports = VeronicaStatusReport;
