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

class InitHelicalTrajectoryRequest {
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
      this.n_turns = null;
      this.delta_z = null;
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
      if (initObj.hasOwnProperty('n_turns')) {
        this.n_turns = initObj.n_turns
      }
      else {
        this.n_turns = 0.0;
      }
      if (initObj.hasOwnProperty('delta_z')) {
        this.delta_z = initObj.delta_z
      }
      else {
        this.delta_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InitHelicalTrajectoryRequest
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
    // Serialize message field [n_turns]
    bufferOffset = _serializer.float64(obj.n_turns, buffer, bufferOffset);
    // Serialize message field [delta_z]
    bufferOffset = _serializer.float64(obj.delta_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InitHelicalTrajectoryRequest
    let len;
    let data = new InitHelicalTrajectoryRequest(null);
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
    // Deserialize message field [n_turns]
    data.n_turns = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_z]
    data.delta_z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 94;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/InitHelicalTrajectoryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cdffc21ee67e10141d55f07c2ab01ebc';
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
    float64 n_turns
    float64 delta_z
    
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
    const resolved = new InitHelicalTrajectoryRequest(null);
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

    if (msg.n_turns !== undefined) {
      resolved.n_turns = msg.n_turns;
    }
    else {
      resolved.n_turns = 0.0
    }

    if (msg.delta_z !== undefined) {
      resolved.delta_z = msg.delta_z;
    }
    else {
      resolved.delta_z = 0.0
    }

    return resolved;
    }
};

class InitHelicalTrajectoryResponse {
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
    // Serializes a message object of type InitHelicalTrajectoryResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InitHelicalTrajectoryResponse
    let len;
    let data = new InitHelicalTrajectoryResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/InitHelicalTrajectoryResponse';
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
    const resolved = new InitHelicalTrajectoryResponse(null);
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
  Request: InitHelicalTrajectoryRequest,
  Response: InitHelicalTrajectoryResponse,
  md5sum() { return 'bae09a54d9b06eeca193015644eeb493'; },
  datatype() { return 'uuv_control_msgs/InitHelicalTrajectory'; }
};
