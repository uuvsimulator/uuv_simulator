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

class SetCurrentVelocityRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.velocity = null;
      this.horizontal_angle = null;
      this.vertical_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0.0;
      }
      if (initObj.hasOwnProperty('horizontal_angle')) {
        this.horizontal_angle = initObj.horizontal_angle
      }
      else {
        this.horizontal_angle = 0.0;
      }
      if (initObj.hasOwnProperty('vertical_angle')) {
        this.vertical_angle = initObj.vertical_angle
      }
      else {
        this.vertical_angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCurrentVelocityRequest
    // Serialize message field [velocity]
    bufferOffset = _serializer.float64(obj.velocity, buffer, bufferOffset);
    // Serialize message field [horizontal_angle]
    bufferOffset = _serializer.float64(obj.horizontal_angle, buffer, bufferOffset);
    // Serialize message field [vertical_angle]
    bufferOffset = _serializer.float64(obj.vertical_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCurrentVelocityRequest
    let len;
    let data = new SetCurrentVelocityRequest(null);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [horizontal_angle]
    data.horizontal_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vertical_angle]
    data.vertical_angle = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_world_ros_plugins_msgs/SetCurrentVelocityRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '44059aaf9c13a2ec083fad30e8a17ee3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    float64 velocity
    float64 horizontal_angle
    float64 vertical_angle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetCurrentVelocityRequest(null);
    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0.0
    }

    if (msg.horizontal_angle !== undefined) {
      resolved.horizontal_angle = msg.horizontal_angle;
    }
    else {
      resolved.horizontal_angle = 0.0
    }

    if (msg.vertical_angle !== undefined) {
      resolved.vertical_angle = msg.vertical_angle;
    }
    else {
      resolved.vertical_angle = 0.0
    }

    return resolved;
    }
};

class SetCurrentVelocityResponse {
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
    // Serializes a message object of type SetCurrentVelocityResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCurrentVelocityResponse
    let len;
    let data = new SetCurrentVelocityResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_world_ros_plugins_msgs/SetCurrentVelocityResponse';
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
    const resolved = new SetCurrentVelocityResponse(null);
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
  Request: SetCurrentVelocityRequest,
  Response: SetCurrentVelocityResponse,
  md5sum() { return '3389770cff5466e5c98d6200f7909bd7'; },
  datatype() { return 'uuv_world_ros_plugins_msgs/SetCurrentVelocity'; }
};
