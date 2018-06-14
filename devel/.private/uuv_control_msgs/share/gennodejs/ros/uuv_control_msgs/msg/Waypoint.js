// Auto-generated. Do not edit!

// (in-package uuv_control_msgs.msg)


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

class Waypoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.point = null;
      this.max_forward_speed = null;
      this.heading_offset = null;
      this.use_fixed_heading = null;
      this.radius_of_acceptance = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = new geometry_msgs.msg.Point();
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
      if (initObj.hasOwnProperty('use_fixed_heading')) {
        this.use_fixed_heading = initObj.use_fixed_heading
      }
      else {
        this.use_fixed_heading = false;
      }
      if (initObj.hasOwnProperty('radius_of_acceptance')) {
        this.radius_of_acceptance = initObj.radius_of_acceptance
      }
      else {
        this.radius_of_acceptance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Waypoint
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.point, buffer, bufferOffset);
    // Serialize message field [max_forward_speed]
    bufferOffset = _serializer.float64(obj.max_forward_speed, buffer, bufferOffset);
    // Serialize message field [heading_offset]
    bufferOffset = _serializer.float64(obj.heading_offset, buffer, bufferOffset);
    // Serialize message field [use_fixed_heading]
    bufferOffset = _serializer.bool(obj.use_fixed_heading, buffer, bufferOffset);
    // Serialize message field [radius_of_acceptance]
    bufferOffset = _serializer.float64(obj.radius_of_acceptance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Waypoint
    let len;
    let data = new Waypoint(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [point]
    data.point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [max_forward_speed]
    data.max_forward_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading_offset]
    data.heading_offset = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [use_fixed_heading]
    data.use_fixed_heading = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [radius_of_acceptance]
    data.radius_of_acceptance = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 49;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uuv_control_msgs/Waypoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0efb7fda1b5980152de94b6064a5cf35';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Waypoint(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.point !== undefined) {
      resolved.point = geometry_msgs.msg.Point.Resolve(msg.point)
    }
    else {
      resolved.point = new geometry_msgs.msg.Point()
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

    if (msg.use_fixed_heading !== undefined) {
      resolved.use_fixed_heading = msg.use_fixed_heading;
    }
    else {
      resolved.use_fixed_heading = false
    }

    if (msg.radius_of_acceptance !== undefined) {
      resolved.radius_of_acceptance = msg.radius_of_acceptance;
    }
    else {
      resolved.radius_of_acceptance = 0.0
    }

    return resolved;
    }
};

module.exports = Waypoint;
