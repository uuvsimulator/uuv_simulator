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

class SetPIDParamsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Kp = null;
      this.Kd = null;
      this.Ki = null;
    }
    else {
      if (initObj.hasOwnProperty('Kp')) {
        this.Kp = initObj.Kp
      }
      else {
        this.Kp = [];
      }
      if (initObj.hasOwnProperty('Kd')) {
        this.Kd = initObj.Kd
      }
      else {
        this.Kd = [];
      }
      if (initObj.hasOwnProperty('Ki')) {
        this.Ki = initObj.Ki
      }
      else {
        this.Ki = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetPIDParamsRequest
    // Serialize message field [Kp]
    bufferOffset = _arraySerializer.float64(obj.Kp, buffer, bufferOffset, null);
    // Serialize message field [Kd]
    bufferOffset = _arraySerializer.float64(obj.Kd, buffer, bufferOffset, null);
    // Serialize message field [Ki]
    bufferOffset = _arraySerializer.float64(obj.Ki, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetPIDParamsRequest
    let len;
    let data = new SetPIDParamsRequest(null);
    // Deserialize message field [Kp]
    data.Kp = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [Kd]
    data.Kd = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [Ki]
    data.Ki = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.Kp.length;
    length += 8 * object.Kd.length;
    length += 8 * object.Ki.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/SetPIDParamsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1dae001799e4bc231c788fb194cf733a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    float64[] Kp
    float64[] Kd
    float64[] Ki
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetPIDParamsRequest(null);
    if (msg.Kp !== undefined) {
      resolved.Kp = msg.Kp;
    }
    else {
      resolved.Kp = []
    }

    if (msg.Kd !== undefined) {
      resolved.Kd = msg.Kd;
    }
    else {
      resolved.Kd = []
    }

    if (msg.Ki !== undefined) {
      resolved.Ki = msg.Ki;
    }
    else {
      resolved.Ki = []
    }

    return resolved;
    }
};

class SetPIDParamsResponse {
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
    // Serializes a message object of type SetPIDParamsResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetPIDParamsResponse
    let len;
    let data = new SetPIDParamsResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/SetPIDParamsResponse';
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
    const resolved = new SetPIDParamsResponse(null);
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
  Request: SetPIDParamsRequest,
  Response: SetPIDParamsResponse,
  md5sum() { return '147ee245a68427e8ed98870bce36b399'; },
  datatype() { return 'uuv_control_msgs/SetPIDParams'; }
};
