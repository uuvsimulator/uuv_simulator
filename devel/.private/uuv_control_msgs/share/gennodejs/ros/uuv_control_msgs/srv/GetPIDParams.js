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

class GetPIDParamsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPIDParamsRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPIDParamsRequest
    let len;
    let data = new GetPIDParamsRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/GetPIDParamsRequest';
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
    const resolved = new GetPIDParamsRequest(null);
    return resolved;
    }
};

class GetPIDParamsResponse {
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
    // Serializes a message object of type GetPIDParamsResponse
    // Serialize message field [Kp]
    bufferOffset = _arraySerializer.float64(obj.Kp, buffer, bufferOffset, null);
    // Serialize message field [Kd]
    bufferOffset = _arraySerializer.float64(obj.Kd, buffer, bufferOffset, null);
    // Serialize message field [Ki]
    bufferOffset = _arraySerializer.float64(obj.Ki, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPIDParamsResponse
    let len;
    let data = new GetPIDParamsResponse(null);
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
    return 'uuv_control_msgs/GetPIDParamsResponse';
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
    const resolved = new GetPIDParamsResponse(null);
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

module.exports = {
  Request: GetPIDParamsRequest,
  Response: GetPIDParamsResponse,
  md5sum() { return '1dae001799e4bc231c788fb194cf733a'; },
  datatype() { return 'uuv_control_msgs/GetPIDParams'; }
};
