// Auto-generated. Do not edit!

// (in-package uuv_control_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class IsRunningTrajectoryRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IsRunningTrajectoryRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IsRunningTrajectoryRequest
    let len;
    let data = new IsRunningTrajectoryRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/IsRunningTrajectoryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IsRunningTrajectoryRequest(null);
    return resolved;
    }
};

class IsRunningTrajectoryResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.progress = null;
    }
    else {
      if (initObj.hasOwnProperty('progress')) {
        this.progress = initObj.progress
      }
      else {
        this.progress = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IsRunningTrajectoryResponse
    // Serialize message field [progress]
    bufferOffset = _serializer.float64(obj.progress, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IsRunningTrajectoryResponse
    let len;
    let data = new IsRunningTrajectoryResponse(null);
    // Deserialize message field [progress]
    data.progress = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/IsRunningTrajectoryResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7afca0099e0cddc25243b1e3569895fe';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 progress
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IsRunningTrajectoryResponse(null);
    if (msg.progress !== undefined) {
      resolved.progress = msg.progress;
    }
    else {
      resolved.progress = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: IsRunningTrajectoryRequest,
  Response: IsRunningTrajectoryResponse,
  md5sum() { return '7afca0099e0cddc25243b1e3569895fe'; },
  datatype() { return 'uuv_control_msgs/IsRunningTrajectory'; }
};
