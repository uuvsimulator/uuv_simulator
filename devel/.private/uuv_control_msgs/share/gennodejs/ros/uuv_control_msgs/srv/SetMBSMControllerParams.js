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

class SetMBSMControllerParamsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lambda = null;
      this.rho_constant = null;
      this.k = null;
      this.c = null;
      this.adapt_slope = null;
      this.rho_0 = null;
      this.drift_prevent = null;
    }
    else {
      if (initObj.hasOwnProperty('lambda')) {
        this.lambda = initObj.lambda
      }
      else {
        this.lambda = [];
      }
      if (initObj.hasOwnProperty('rho_constant')) {
        this.rho_constant = initObj.rho_constant
      }
      else {
        this.rho_constant = [];
      }
      if (initObj.hasOwnProperty('k')) {
        this.k = initObj.k
      }
      else {
        this.k = [];
      }
      if (initObj.hasOwnProperty('c')) {
        this.c = initObj.c
      }
      else {
        this.c = [];
      }
      if (initObj.hasOwnProperty('adapt_slope')) {
        this.adapt_slope = initObj.adapt_slope
      }
      else {
        this.adapt_slope = [];
      }
      if (initObj.hasOwnProperty('rho_0')) {
        this.rho_0 = initObj.rho_0
      }
      else {
        this.rho_0 = [];
      }
      if (initObj.hasOwnProperty('drift_prevent')) {
        this.drift_prevent = initObj.drift_prevent
      }
      else {
        this.drift_prevent = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetMBSMControllerParamsRequest
    // Serialize message field [lambda]
    bufferOffset = _arraySerializer.float64(obj.lambda, buffer, bufferOffset, null);
    // Serialize message field [rho_constant]
    bufferOffset = _arraySerializer.float64(obj.rho_constant, buffer, bufferOffset, null);
    // Serialize message field [k]
    bufferOffset = _arraySerializer.float64(obj.k, buffer, bufferOffset, null);
    // Serialize message field [c]
    bufferOffset = _arraySerializer.float64(obj.c, buffer, bufferOffset, null);
    // Serialize message field [adapt_slope]
    bufferOffset = _arraySerializer.float64(obj.adapt_slope, buffer, bufferOffset, null);
    // Serialize message field [rho_0]
    bufferOffset = _arraySerializer.float64(obj.rho_0, buffer, bufferOffset, null);
    // Serialize message field [drift_prevent]
    bufferOffset = _serializer.float64(obj.drift_prevent, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetMBSMControllerParamsRequest
    let len;
    let data = new SetMBSMControllerParamsRequest(null);
    // Deserialize message field [lambda]
    data.lambda = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [rho_constant]
    data.rho_constant = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [k]
    data.k = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [c]
    data.c = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [adapt_slope]
    data.adapt_slope = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [rho_0]
    data.rho_0 = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [drift_prevent]
    data.drift_prevent = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.lambda.length;
    length += 8 * object.rho_constant.length;
    length += 8 * object.k.length;
    length += 8 * object.c.length;
    length += 8 * object.adapt_slope.length;
    length += 8 * object.rho_0.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/SetMBSMControllerParamsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7293aecc8487ffe3e998814d65aa6940';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    float64[] lambda
    float64[] rho_constant
    float64[] k
    float64[] c
    float64[] adapt_slope
    float64[] rho_0
    float64 drift_prevent
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetMBSMControllerParamsRequest(null);
    if (msg.lambda !== undefined) {
      resolved.lambda = msg.lambda;
    }
    else {
      resolved.lambda = []
    }

    if (msg.rho_constant !== undefined) {
      resolved.rho_constant = msg.rho_constant;
    }
    else {
      resolved.rho_constant = []
    }

    if (msg.k !== undefined) {
      resolved.k = msg.k;
    }
    else {
      resolved.k = []
    }

    if (msg.c !== undefined) {
      resolved.c = msg.c;
    }
    else {
      resolved.c = []
    }

    if (msg.adapt_slope !== undefined) {
      resolved.adapt_slope = msg.adapt_slope;
    }
    else {
      resolved.adapt_slope = []
    }

    if (msg.rho_0 !== undefined) {
      resolved.rho_0 = msg.rho_0;
    }
    else {
      resolved.rho_0 = []
    }

    if (msg.drift_prevent !== undefined) {
      resolved.drift_prevent = msg.drift_prevent;
    }
    else {
      resolved.drift_prevent = 0.0
    }

    return resolved;
    }
};

class SetMBSMControllerParamsResponse {
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
    // Serializes a message object of type SetMBSMControllerParamsResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetMBSMControllerParamsResponse
    let len;
    let data = new SetMBSMControllerParamsResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/SetMBSMControllerParamsResponse';
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
    const resolved = new SetMBSMControllerParamsResponse(null);
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
  Request: SetMBSMControllerParamsRequest,
  Response: SetMBSMControllerParamsResponse,
  md5sum() { return 'a72093ed49272680945c74a3bdff401c'; },
  datatype() { return 'uuv_control_msgs/SetMBSMControllerParams'; }
};
