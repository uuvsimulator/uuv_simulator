// Auto-generated. Do not edit!

// (in-package uuv_world_ros_plugins_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetCurrentModelRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mean = null;
      this.min = null;
      this.max = null;
      this.noise = null;
      this.mu = null;
    }
    else {
      if (initObj.hasOwnProperty('mean')) {
        this.mean = initObj.mean
      }
      else {
        this.mean = 0.0;
      }
      if (initObj.hasOwnProperty('min')) {
        this.min = initObj.min
      }
      else {
        this.min = 0.0;
      }
      if (initObj.hasOwnProperty('max')) {
        this.max = initObj.max
      }
      else {
        this.max = 0.0;
      }
      if (initObj.hasOwnProperty('noise')) {
        this.noise = initObj.noise
      }
      else {
        this.noise = 0.0;
      }
      if (initObj.hasOwnProperty('mu')) {
        this.mu = initObj.mu
      }
      else {
        this.mu = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCurrentModelRequest
    // Serialize message field [mean]
    bufferOffset = _serializer.float64(obj.mean, buffer, bufferOffset);
    // Serialize message field [min]
    bufferOffset = _serializer.float64(obj.min, buffer, bufferOffset);
    // Serialize message field [max]
    bufferOffset = _serializer.float64(obj.max, buffer, bufferOffset);
    // Serialize message field [noise]
    bufferOffset = _serializer.float64(obj.noise, buffer, bufferOffset);
    // Serialize message field [mu]
    bufferOffset = _serializer.float64(obj.mu, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCurrentModelRequest
    let len;
    let data = new SetCurrentModelRequest(null);
    // Deserialize message field [mean]
    data.mean = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [min]
    data.min = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max]
    data.max = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [noise]
    data.noise = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mu]
    data.mu = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_world_ros_plugins_msgs/SetCurrentModelRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b8222571af4e4180b9b706d1e17ad7e1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    float64 mean
    float64 min
    float64 max
    float64 noise
    float64 mu
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetCurrentModelRequest(null);
    if (msg.mean !== undefined) {
      resolved.mean = msg.mean;
    }
    else {
      resolved.mean = 0.0
    }

    if (msg.min !== undefined) {
      resolved.min = msg.min;
    }
    else {
      resolved.min = 0.0
    }

    if (msg.max !== undefined) {
      resolved.max = msg.max;
    }
    else {
      resolved.max = 0.0
    }

    if (msg.noise !== undefined) {
      resolved.noise = msg.noise;
    }
    else {
      resolved.noise = 0.0
    }

    if (msg.mu !== undefined) {
      resolved.mu = msg.mu;
    }
    else {
      resolved.mu = 0.0
    }

    return resolved;
    }
};

class SetCurrentModelResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCurrentModelResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCurrentModelResponse
    let len;
    let data = new SetCurrentModelResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_world_ros_plugins_msgs/SetCurrentModelResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetCurrentModelResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SetCurrentModelRequest,
  Response: SetCurrentModelResponse,
  md5sum() { return '97b431fd7a0d7100472976b98c315e14'; },
  datatype() { return 'uuv_world_ros_plugins_msgs/SetCurrentModel'; }
};
