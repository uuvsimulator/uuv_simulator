// Auto-generated. Do not edit!

// (in-package uuv_control_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class InitWaypointsFromFileRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_time = null;
      this.start_now = null;
      this.filename = null;
      this.interpolator = null;
    }
    else {
      if (initObj.hasOwnProperty('start_time')) {
        this.start_time = initObj.start_time
      }
      else {
        this.start_time = new std_msgs.msg.Time();
      }
      if (initObj.hasOwnProperty('start_now')) {
        this.start_now = initObj.start_now
      }
      else {
        this.start_now = false;
      }
      if (initObj.hasOwnProperty('filename')) {
        this.filename = initObj.filename
      }
      else {
        this.filename = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('interpolator')) {
        this.interpolator = initObj.interpolator
      }
      else {
        this.interpolator = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InitWaypointsFromFileRequest
    // Serialize message field [start_time]
    bufferOffset = std_msgs.msg.Time.serialize(obj.start_time, buffer, bufferOffset);
    // Serialize message field [start_now]
    bufferOffset = _serializer.bool(obj.start_now, buffer, bufferOffset);
    // Serialize message field [filename]
    bufferOffset = std_msgs.msg.String.serialize(obj.filename, buffer, bufferOffset);
    // Serialize message field [interpolator]
    bufferOffset = std_msgs.msg.String.serialize(obj.interpolator, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InitWaypointsFromFileRequest
    let len;
    let data = new InitWaypointsFromFileRequest(null);
    // Deserialize message field [start_time]
    data.start_time = std_msgs.msg.Time.deserialize(buffer, bufferOffset);
    // Deserialize message field [start_now]
    data.start_now = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [filename]
    data.filename = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [interpolator]
    data.interpolator = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.filename);
    length += std_msgs.msg.String.getMessageSize(object.interpolator);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/InitWaypointsFromFileRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ddaf659bb5628d87c763d02c9d4cc76b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    string LIPB='lipb'
    string CUBIC='cubic'
    string DUBINS='dubins'
    string LINEAR='linear'
    
    std_msgs/Time start_time
    bool start_now
    std_msgs/String filename
    std_msgs/String interpolator
    
    ================================================================================
    MSG: std_msgs/Time
    time data
    
    ================================================================================
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InitWaypointsFromFileRequest(null);
    if (msg.start_time !== undefined) {
      resolved.start_time = std_msgs.msg.Time.Resolve(msg.start_time)
    }
    else {
      resolved.start_time = new std_msgs.msg.Time()
    }

    if (msg.start_now !== undefined) {
      resolved.start_now = msg.start_now;
    }
    else {
      resolved.start_now = false
    }

    if (msg.filename !== undefined) {
      resolved.filename = std_msgs.msg.String.Resolve(msg.filename)
    }
    else {
      resolved.filename = new std_msgs.msg.String()
    }

    if (msg.interpolator !== undefined) {
      resolved.interpolator = std_msgs.msg.String.Resolve(msg.interpolator)
    }
    else {
      resolved.interpolator = new std_msgs.msg.String()
    }

    return resolved;
    }
};

// Constants for message
InitWaypointsFromFileRequest.Constants = {
  LIPB: ''lipb'',
  CUBIC: ''cubic'',
  DUBINS: ''dubins'',
  LINEAR: ''linear'',
}

class InitWaypointsFromFileResponse {
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
    // Serializes a message object of type InitWaypointsFromFileResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InitWaypointsFromFileResponse
    let len;
    let data = new InitWaypointsFromFileResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/InitWaypointsFromFileResponse';
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
    const resolved = new InitWaypointsFromFileResponse(null);
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
  Request: InitWaypointsFromFileRequest,
  Response: InitWaypointsFromFileResponse,
  md5sum() { return 'a0198a3f7b74ab5dc081e88db6d85c58'; },
  datatype() { return 'uuv_control_msgs/InitWaypointsFromFile'; }
};
