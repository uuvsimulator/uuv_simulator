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

//-----------------------------------------------------------


//-----------------------------------------------------------

class GoToRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.waypoint = null;
      this.max_forward_speed = null;
      this.interpolator = null;
    }
    else {
      if (initObj.hasOwnProperty('waypoint')) {
        this.waypoint = initObj.waypoint
      }
      else {
        this.waypoint = new Waypoint();
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
    // Serializes a message object of type GoToRequest
    // Serialize message field [waypoint]
    bufferOffset = Waypoint.serialize(obj.waypoint, buffer, bufferOffset);
    // Serialize message field [max_forward_speed]
    bufferOffset = _serializer.float64(obj.max_forward_speed, buffer, bufferOffset);
    // Serialize message field [interpolator]
    bufferOffset = _serializer.string(obj.interpolator, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GoToRequest
    let len;
    let data = new GoToRequest(null);
    // Deserialize message field [waypoint]
    data.waypoint = Waypoint.deserialize(buffer, bufferOffset);
    // Deserialize message field [max_forward_speed]
    data.max_forward_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [interpolator]
    data.interpolator = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Waypoint.getMessageSize(object.waypoint);
    length += object.interpolator.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/GoToRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ca7b4067b6783823c9f5c936e1c99b3a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    string LIPB='lipb'
    string CUBIC='cubic'
    string DUBINS='dubins'
    string LINEAR='linear'
    
    uuv_control_msgs/Waypoint waypoint
    float64 max_forward_speed
    string interpolator
    
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
    const resolved = new GoToRequest(null);
    if (msg.waypoint !== undefined) {
      resolved.waypoint = Waypoint.Resolve(msg.waypoint)
    }
    else {
      resolved.waypoint = new Waypoint()
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
GoToRequest.Constants = {
  LIPB: ''lipb'',
  CUBIC: ''cubic'',
  DUBINS: ''dubins'',
  LINEAR: ''linear'',
}

class GoToResponse {
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
    // Serializes a message object of type GoToResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GoToResponse
    let len;
    let data = new GoToResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_control_msgs/GoToResponse';
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
    const resolved = new GoToResponse(null);
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
  Request: GoToRequest,
  Response: GoToResponse,
  md5sum() { return '408446fa9ec1d90b38a7053e3dd0ad47'; },
  datatype() { return 'uuv_control_msgs/GoTo'; }
};
