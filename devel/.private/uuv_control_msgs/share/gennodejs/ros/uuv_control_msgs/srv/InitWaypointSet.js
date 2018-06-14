// Auto-generated. Do not edit!

// (in-package uuv_control_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Waypoint = require('../msg/Waypoint.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class InitWaypointSetRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_time = null;
      this.start_now = null;
      this.waypoints = null;
      this.max_forward_speed = null;
      this.heading_offset = null;
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
      if (initObj.hasOwnProperty('waypoints')) {
        this.waypoints = initObj.waypoints
      }
      else {
        this.waypoints = [];
      }
      if (initObj.hasOwnProperty('max_forward_speed')) {
        this.max_forward_speed = initObj.max_forward_speed
      }
      else {
        this.max_forward_speed = 0.0;
      }
      if (initObj.hasOwnProperty('heading_offset')) {
        this.heading_offset = initObj.heading_offset
      }
      else {
        this.heading_offset = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InitWaypointSetRequest
    // Serialize message field [start_time]
    bufferOffset = std_msgs.msg.Time.serialize(obj.start_time, buffer, bufferOffset);
    // Serialize message field [start_now]
    bufferOffset = _serializer.bool(obj.start_now, buffer, bufferOffset);
    // Serialize message field [waypoints]
    // Serialize the length for message field [waypoints]
    bufferOffset = _serializer.uint32(obj.waypoints.length, buffer, bufferOffset);
    obj.waypoints.forEach((val) => {
      bufferOffset = Waypoint.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [max_forward_speed]
    bufferOffset = _serializer.float64(obj.max_forward_speed, buffer, bufferOffset);
    // Serialize message field [heading_offset]
    bufferOffset = _serializer.float64(obj.heading_offset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InitWaypointSetRequest
    let len;
    let data = new InitWaypointSetRequest(null);
    // Deserialize message field [start_time]
    data.start_time = std_msgs.msg.Time.deserialize(buffer, bufferOffset);
    // Deserialize message field [start_now]
    data.start_now = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [waypoints]
    // Deserialize array length for message field [waypoints]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.waypoints = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.waypoints[i] = Waypoint.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [max_forward_speed]
    data.max_forward_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading_offset]
    data.heading_offset = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.waypoints.forEach((val) => {
      length += Waypoint.getMessageSize(val);
    });
    return length + 29;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/InitWaypointSetRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cdbe1608ecbda3cf21f7ed8b675d66a8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    std_msgs/Time start_time
    bool start_now
    uuv_control_msgs/Waypoint[] waypoints
    float64 max_forward_speed
    float64 heading_offset
    
    ================================================================================
    MSG: std_msgs/Time
    time data
    
    ================================================================================
    MSG: uuv_control_msgs/Waypoint
    # Copyright (c) 2016 The UUV Simulator Authors.
    # All rights reserved.
    #
    # Licensed under the Apache License, Version 2.0 (the "License");
    # you may not use this file except in compliance with the License.
    # You may obtain a copy of the License at
    #
    #     http://www.apache.org/licenses/LICENSE-2.0
    #
    # Unless required by applicable law or agreed to in writing, software
    # distributed under the License is distributed on an "AS IS" BASIS,
    # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    # See the License for the specific language governing permissions and
    # limitations under the License.
    
    std_msgs/Header header
    geometry_msgs/Point point
    float64 max_forward_speed
    float64 heading_offset
    bool use_fixed_heading
    float64 radius_of_acceptance
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
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
    const resolved = new InitWaypointSetRequest(null);
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

    if (msg.waypoints !== undefined) {
      resolved.waypoints = new Array(msg.waypoints.length);
      for (let i = 0; i < resolved.waypoints.length; ++i) {
        resolved.waypoints[i] = Waypoint.Resolve(msg.waypoints[i]);
      }
    }
    else {
      resolved.waypoints = []
    }

    if (msg.max_forward_speed !== undefined) {
      resolved.max_forward_speed = msg.max_forward_speed;
    }
    else {
      resolved.max_forward_speed = 0.0
    }

    if (msg.heading_offset !== undefined) {
      resolved.heading_offset = msg.heading_offset;
    }
    else {
      resolved.heading_offset = 0.0
    }

    return resolved;
    }
};

class InitWaypointSetResponse {
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
    // Serializes a message object of type InitWaypointSetResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InitWaypointSetResponse
    let len;
    let data = new InitWaypointSetResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/InitWaypointSetResponse';
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
    const resolved = new InitWaypointSetResponse(null);
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
  Request: InitWaypointSetRequest,
  Response: InitWaypointSetResponse,
  md5sum() { return 'a5fe64b7879e685af95ae8210e45b1c6'; },
  datatype() { return 'uuv_control_msgs/InitWaypointSet'; }
};
