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

class InitCircularTrajectoryRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_time = null;
      this.start_now = null;
      this.radius = null;
      this.center = null;
      this.is_clockwise = null;
      this.angle_offset = null;
      this.n_points = null;
      this.heading_offset = null;
      this.max_forward_speed = null;
      this.duration = null;
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
      if (initObj.hasOwnProperty('radius')) {
        this.radius = initObj.radius
      }
      else {
        this.radius = 0.0;
      }
      if (initObj.hasOwnProperty('center')) {
        this.center = initObj.center
      }
      else {
        this.center = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('is_clockwise')) {
        this.is_clockwise = initObj.is_clockwise
      }
      else {
        this.is_clockwise = false;
      }
      if (initObj.hasOwnProperty('angle_offset')) {
        this.angle_offset = initObj.angle_offset
      }
      else {
        this.angle_offset = 0.0;
      }
      if (initObj.hasOwnProperty('n_points')) {
        this.n_points = initObj.n_points
      }
      else {
        this.n_points = 0;
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
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InitCircularTrajectoryRequest
    // Serialize message field [start_time]
    bufferOffset = std_msgs.msg.Time.serialize(obj.start_time, buffer, bufferOffset);
    // Serialize message field [start_now]
    bufferOffset = _serializer.bool(obj.start_now, buffer, bufferOffset);
    // Serialize message field [radius]
    bufferOffset = _serializer.float64(obj.radius, buffer, bufferOffset);
    // Serialize message field [center]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.center, buffer, bufferOffset);
    // Serialize message field [is_clockwise]
    bufferOffset = _serializer.bool(obj.is_clockwise, buffer, bufferOffset);
    // Serialize message field [angle_offset]
    bufferOffset = _serializer.float64(obj.angle_offset, buffer, bufferOffset);
    // Serialize message field [n_points]
    bufferOffset = _serializer.int32(obj.n_points, buffer, bufferOffset);
    // Serialize message field [heading_offset]
    bufferOffset = _serializer.float64(obj.heading_offset, buffer, bufferOffset);
    // Serialize message field [max_forward_speed]
    bufferOffset = _serializer.float64(obj.max_forward_speed, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.float64(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InitCircularTrajectoryRequest
    let len;
    let data = new InitCircularTrajectoryRequest(null);
    // Deserialize message field [start_time]
    data.start_time = std_msgs.msg.Time.deserialize(buffer, bufferOffset);
    // Deserialize message field [start_now]
    data.start_now = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [radius]
    data.radius = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [center]
    data.center = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [is_clockwise]
    data.is_clockwise = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [angle_offset]
    data.angle_offset = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [n_points]
    data.n_points = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [heading_offset]
    data.heading_offset = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_forward_speed]
    data.max_forward_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 78;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/InitCircularTrajectoryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '33f617e6e74b9a5a4089105d4a0a3b2f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    std_msgs/Time start_time
    bool start_now
    float64 radius
    geometry_msgs/Point center
    bool is_clockwise
    float64 angle_offset
    int32 n_points
    float64 heading_offset
    float64 max_forward_speed
    float64 duration
    
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
    const resolved = new InitCircularTrajectoryRequest(null);
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

    if (msg.radius !== undefined) {
      resolved.radius = msg.radius;
    }
    else {
      resolved.radius = 0.0
    }

    if (msg.center !== undefined) {
      resolved.center = geometry_msgs.msg.Point.Resolve(msg.center)
    }
    else {
      resolved.center = new geometry_msgs.msg.Point()
    }

    if (msg.is_clockwise !== undefined) {
      resolved.is_clockwise = msg.is_clockwise;
    }
    else {
      resolved.is_clockwise = false
    }

    if (msg.angle_offset !== undefined) {
      resolved.angle_offset = msg.angle_offset;
    }
    else {
      resolved.angle_offset = 0.0
    }

    if (msg.n_points !== undefined) {
      resolved.n_points = msg.n_points;
    }
    else {
      resolved.n_points = 0
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

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0.0
    }

    return resolved;
    }
};

class InitCircularTrajectoryResponse {
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
    // Serializes a message object of type InitCircularTrajectoryResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InitCircularTrajectoryResponse
    let len;
    let data = new InitCircularTrajectoryResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/InitCircularTrajectoryResponse';
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
    const resolved = new InitCircularTrajectoryResponse(null);
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
  Request: InitCircularTrajectoryRequest,
  Response: InitCircularTrajectoryResponse,
  md5sum() { return '4e2b6d8506f8a3b1f6ffe498d85ccc39'; },
  datatype() { return 'uuv_control_msgs/InitCircularTrajectory'; }
};
