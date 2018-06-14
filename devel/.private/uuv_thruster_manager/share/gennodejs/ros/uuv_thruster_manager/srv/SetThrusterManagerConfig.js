// Auto-generated. Do not edit!

// (in-package uuv_thruster_manager.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetThrusterManagerConfigRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.base_link = null;
      this.thruster_frame_base = null;
      this.thruster_topic_prefix = null;
      this.thruster_topic_suffix = null;
      this.timeout = null;
      this.max_thrust = null;
    }
    else {
      if (initObj.hasOwnProperty('base_link')) {
        this.base_link = initObj.base_link
      }
      else {
        this.base_link = '';
      }
      if (initObj.hasOwnProperty('thruster_frame_base')) {
        this.thruster_frame_base = initObj.thruster_frame_base
      }
      else {
        this.thruster_frame_base = '';
      }
      if (initObj.hasOwnProperty('thruster_topic_prefix')) {
        this.thruster_topic_prefix = initObj.thruster_topic_prefix
      }
      else {
        this.thruster_topic_prefix = '';
      }
      if (initObj.hasOwnProperty('thruster_topic_suffix')) {
        this.thruster_topic_suffix = initObj.thruster_topic_suffix
      }
      else {
        this.thruster_topic_suffix = '';
      }
      if (initObj.hasOwnProperty('timeout')) {
        this.timeout = initObj.timeout
      }
      else {
        this.timeout = 0.0;
      }
      if (initObj.hasOwnProperty('max_thrust')) {
        this.max_thrust = initObj.max_thrust
      }
      else {
        this.max_thrust = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetThrusterManagerConfigRequest
    // Serialize message field [base_link]
    bufferOffset = _serializer.string(obj.base_link, buffer, bufferOffset);
    // Serialize message field [thruster_frame_base]
    bufferOffset = _serializer.string(obj.thruster_frame_base, buffer, bufferOffset);
    // Serialize message field [thruster_topic_prefix]
    bufferOffset = _serializer.string(obj.thruster_topic_prefix, buffer, bufferOffset);
    // Serialize message field [thruster_topic_suffix]
    bufferOffset = _serializer.string(obj.thruster_topic_suffix, buffer, bufferOffset);
    // Serialize message field [timeout]
    bufferOffset = _serializer.float64(obj.timeout, buffer, bufferOffset);
    // Serialize message field [max_thrust]
    bufferOffset = _serializer.float64(obj.max_thrust, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetThrusterManagerConfigRequest
    let len;
    let data = new SetThrusterManagerConfigRequest(null);
    // Deserialize message field [base_link]
    data.base_link = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [thruster_frame_base]
    data.thruster_frame_base = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [thruster_topic_prefix]
    data.thruster_topic_prefix = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [thruster_topic_suffix]
    data.thruster_topic_suffix = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [timeout]
    data.timeout = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_thrust]
    data.max_thrust = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.base_link.length;
    length += object.thruster_frame_base.length;
    length += object.thruster_topic_prefix.length;
    length += object.thruster_topic_suffix.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_thruster_manager/SetThrusterManagerConfigRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '148002d6fd6bf6684e854899710838a2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    string DEFAULT_BASE_LINK            = /base_link
    string DEFAULT_THRUSTER_FRAME_BASE  = /thruster_
    string DEFAULT_PREFIX               = thrusters/
    string DEFAULT_SUFFIX               = /input
    
    string base_link
    string thruster_frame_base
    string thruster_topic_prefix
    string thruster_topic_suffix
    float64 timeout
    float64 max_thrust
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetThrusterManagerConfigRequest(null);
    if (msg.base_link !== undefined) {
      resolved.base_link = msg.base_link;
    }
    else {
      resolved.base_link = ''
    }

    if (msg.thruster_frame_base !== undefined) {
      resolved.thruster_frame_base = msg.thruster_frame_base;
    }
    else {
      resolved.thruster_frame_base = ''
    }

    if (msg.thruster_topic_prefix !== undefined) {
      resolved.thruster_topic_prefix = msg.thruster_topic_prefix;
    }
    else {
      resolved.thruster_topic_prefix = ''
    }

    if (msg.thruster_topic_suffix !== undefined) {
      resolved.thruster_topic_suffix = msg.thruster_topic_suffix;
    }
    else {
      resolved.thruster_topic_suffix = ''
    }

    if (msg.timeout !== undefined) {
      resolved.timeout = msg.timeout;
    }
    else {
      resolved.timeout = 0.0
    }

    if (msg.max_thrust !== undefined) {
      resolved.max_thrust = msg.max_thrust;
    }
    else {
      resolved.max_thrust = 0.0
    }

    return resolved;
    }
};

// Constants for message
SetThrusterManagerConfigRequest.Constants = {
  DEFAULT_BASE_LINK: '/base_link',
  DEFAULT_THRUSTER_FRAME_BASE: '/thruster_',
  DEFAULT_PREFIX: 'thrusters/',
  DEFAULT_SUFFIX: '/input',
}

class SetThrusterManagerConfigResponse {
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
    // Serializes a message object of type SetThrusterManagerConfigResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetThrusterManagerConfigResponse
    let len;
    let data = new SetThrusterManagerConfigResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_thruster_manager/SetThrusterManagerConfigResponse';
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
    const resolved = new SetThrusterManagerConfigResponse(null);
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
  Request: SetThrusterManagerConfigRequest,
  Response: SetThrusterManagerConfigResponse,
  md5sum() { return 'e9f260f9f8a74cbd9cca6d6d276790c0'; },
  datatype() { return 'uuv_thruster_manager/SetThrusterManagerConfig'; }
};
