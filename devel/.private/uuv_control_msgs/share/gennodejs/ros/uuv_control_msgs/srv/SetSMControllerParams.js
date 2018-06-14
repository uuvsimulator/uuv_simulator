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

class SetSMControllerParamsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.K = null;
      this.Kd = null;
      this.Ki = null;
      this.slope = null;
    }
    else {
      if (initObj.hasOwnProperty('K')) {
        this.K = initObj.K
      }
      else {
        this.K = [];
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
      if (initObj.hasOwnProperty('slope')) {
        this.slope = initObj.slope
      }
      else {
        this.slope = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetSMControllerParamsRequest
    // Serialize message field [K]
    bufferOffset = _arraySerializer.float64(obj.K, buffer, bufferOffset, null);
    // Serialize message field [Kd]
    bufferOffset = _arraySerializer.float64(obj.Kd, buffer, bufferOffset, null);
    // Serialize message field [Ki]
    bufferOffset = _arraySerializer.float64(obj.Ki, buffer, bufferOffset, null);
    // Serialize message field [slope]
    bufferOffset = _arraySerializer.float64(obj.slope, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetSMControllerParamsRequest
    let len;
    let data = new SetSMControllerParamsRequest(null);
    // Deserialize message field [K]
    data.K = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [Kd]
    data.Kd = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [Ki]
    data.Ki = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [slope]
    data.slope = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.K.length;
    length += 8 * object.Kd.length;
    length += 8 * object.Ki.length;
    length += 8 * object.slope.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/SetSMControllerParamsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dc294c7929e39ce02ce0ce70a116b3b3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    float64[] K
    float64[] Kd
    float64[] Ki
    float64[] slope
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetSMControllerParamsRequest(null);
    if (msg.K !== undefined) {
      resolved.K = msg.K;
    }
    else {
      resolved.K = []
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

    if (msg.slope !== undefined) {
      resolved.slope = msg.slope;
    }
    else {
      resolved.slope = []
    }

    return resolved;
    }
};

class SetSMControllerParamsResponse {
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
    // Serializes a message object of type SetSMControllerParamsResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetSMControllerParamsResponse
    let len;
    let data = new SetSMControllerParamsResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/SetSMControllerParamsResponse';
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
    const resolved = new SetSMControllerParamsResponse(null);
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
  Request: SetSMControllerParamsRequest,
  Response: SetSMControllerParamsResponse,
  md5sum() { return '14eb4a985748b74cd49f084da4accc8b'; },
  datatype() { return 'uuv_control_msgs/SetSMControllerParams'; }
};
