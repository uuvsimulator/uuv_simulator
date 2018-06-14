// Auto-generated. Do not edit!

// (in-package uuv_control_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class GoToIncrementalRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.step = null;
      this.max_forward_speed = null;
      this.interpolator = null;
    }
    else {
      if (initObj.hasOwnProperty('step')) {
        this.step = initObj.step
      }
      else {
        this.step = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('max_forward_speed')) {
        this.max_forward_speed = initObj.max_forward_speed
      }
      else {
        this.max_forward_speed = 0.0;
      }
      if (initObj.hasOwnProperty('interpolator')) {
        this.interpolator = initObj.interpolator
      }
      else {
        this.interpolator = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GoToIncrementalRequest
    // Serialize message field [step]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.step, buffer, bufferOffset);
    // Serialize message field [max_forward_speed]
    bufferOffset = _serializer.float64(obj.max_forward_speed, buffer, bufferOffset);
    // Serialize message field [interpolator]
    bufferOffset = _serializer.string(obj.interpolator, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GoToIncrementalRequest
    let len;
    let data = new GoToIncrementalRequest(null);
    // Deserialize message field [step]
    data.step = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [max_forward_speed]
    data.max_forward_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [interpolator]
    data.interpolator = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.interpolator.length;
    return length + 36;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/GoToIncrementalRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '83b72c3bd49f592c1f3511f3c61026e3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    string LIPB='lipb'
    string CUBIC='cubic'
    string DUBINS='dubins'
    string LINEAR='linear'
    
    geometry_msgs/Point step
    float64 max_forward_speed
    string interpolator
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GoToIncrementalRequest(null);
    if (msg.step !== undefined) {
      resolved.step = geometry_msgs.msg.Point.Resolve(msg.step)
    }
    else {
      resolved.step = new geometry_msgs.msg.Point()
    }

    if (msg.max_forward_speed !== undefined) {
      resolved.max_forward_speed = msg.max_forward_speed;
    }
    else {
      resolved.max_forward_speed = 0.0
    }

    if (msg.interpolator !== undefined) {
      resolved.interpolator = msg.interpolator;
    }
    else {
      resolved.interpolator = ''
    }

    return resolved;
    }
};

// Constants for message
GoToIncrementalRequest.Constants = {
  LIPB: ''lipb'',
  CUBIC: ''cubic'',
  DUBINS: ''dubins'',
  LINEAR: ''linear'',
}

class GoToIncrementalResponse {
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
    // Serializes a message object of type GoToIncrementalResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GoToIncrementalResponse
    let len;
    let data = new GoToIncrementalResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/GoToIncrementalResponse';
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
    const resolved = new GoToIncrementalResponse(null);
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
  Request: GoToIncrementalRequest,
  Response: GoToIncrementalResponse,
  md5sum() { return 'ea062c779dd21cbac8fefabcd9b5f18e'; },
  datatype() { return 'uuv_control_msgs/GoToIncremental'; }
};
