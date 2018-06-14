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
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class InitRectTrajectoryRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_time = null;
      this.start_now = null;
      this.origin = null;
      this.height = null;
      this.width = null;
      this.angle_offset = null;
      this.heading_offset = null;
      this.max_forward_speed = null;
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
      if (initObj.hasOwnProperty('origin')) {
        this.origin = initObj.origin
      }
      else {
        this.origin = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0.0;
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0.0;
      }
      if (initObj.hasOwnProperty('angle_offset')) {
        this.angle_offset = initObj.angle_offset
      }
      else {
        this.angle_offset = 0.0;
      }
      if (initObj.hasOwnProperty('heading_offset')) {
        this.heading_offset = initObj.heading_offset
      }
      else {
        this.heading_offset = 0.0;
      }
      if (initObj.hasOwnProperty('max_forward_speed')) {
        this.max_forward_speed = initObj.max_forward_speed
      }
      else {
        this.max_forward_speed = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InitRectTrajectoryRequest
    // Serialize message field [start_time]
    bufferOffset = std_msgs.msg.Time.serialize(obj.start_time, buffer, bufferOffset);
    // Serialize message field [start_now]
    bufferOffset = _serializer.bool(obj.start_now, buffer, bufferOffset);
    // Serialize message field [origin]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.origin, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.float64(obj.height, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.float64(obj.width, buffer, bufferOffset);
    // Serialize message field [angle_offset]
    bufferOffset = _serializer.float64(obj.angle_offset, buffer, bufferOffset);
    // Serialize message field [heading_offset]
    bufferOffset = _serializer.float64(obj.heading_offset, buffer, bufferOffset);
    // Serialize message field [max_forward_speed]
    bufferOffset = _serializer.float64(obj.max_forward_speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InitRectTrajectoryRequest
    let len;
    let data = new InitRectTrajectoryRequest(null);
    // Deserialize message field [start_time]
    data.start_time = std_msgs.msg.Time.deserialize(buffer, bufferOffset);
    // Deserialize message field [start_now]
    data.start_now = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [origin]
    data.origin = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angle_offset]
    data.angle_offset = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading_offset]
    data.heading_offset = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_forward_speed]
    data.max_forward_speed = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 73;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/InitRectTrajectoryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '82860775b3df6259cef2af6522eca70e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    std_msgs/Time start_time
    bool start_now
    geometry_msgs/Point origin
    float64 height
    float64 width
    float64 angle_offset
    float64 heading_offset
    float64 max_forward_speed
    
    ================================================================================
    MSG: std_msgs/Time
    time data
    
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
    const resolved = new InitRectTrajectoryRequest(null);
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

    if (msg.origin !== undefined) {
      resolved.origin = geometry_msgs.msg.Point.Resolve(msg.origin)
    }
    else {
      resolved.origin = new geometry_msgs.msg.Point()
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0.0
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0.0
    }

    if (msg.angle_offset !== undefined) {
      resolved.angle_offset = msg.angle_offset;
    }
    else {
      resolved.angle_offset = 0.0
    }

    if (msg.heading_offset !== undefined) {
      resolved.heading_offset = msg.heading_offset;
    }
    else {
      resolved.heading_offset = 0.0
    }

    if (msg.max_forward_speed !== undefined) {
      resolved.max_forward_speed = msg.max_forward_speed;
    }
    else {
      resolved.max_forward_speed = 0.0
    }

    return resolved;
    }
};

class InitRectTrajectoryResponse {
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
    // Serializes a message object of type InitRectTrajectoryResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InitRectTrajectoryResponse
    let len;
    let data = new InitRectTrajectoryResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/InitRectTrajectoryResponse';
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
    const resolved = new InitRectTrajectoryResponse(null);
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
  Request: InitRectTrajectoryRequest,
  Response: InitRectTrajectoryResponse,
  md5sum() { return 'bb6b6b97f153ba237ef24a0678facef1'; },
  datatype() { return 'uuv_control_msgs/InitRectTrajectory'; }
};
