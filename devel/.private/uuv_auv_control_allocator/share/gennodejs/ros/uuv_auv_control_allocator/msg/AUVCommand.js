// Auto-generated. Do not edit!

// (in-package uuv_auv_control_allocator.msg)


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

class AUVCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.surge_speed = null;
      this.command = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('surge_speed')) {
        this.surge_speed = initObj.surge_speed
      }
      else {
        this.surge_speed = 0.0;
      }
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = new geometry_msgs.msg.Wrench();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AUVCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [surge_speed]
    bufferOffset = _serializer.float64(obj.surge_speed, buffer, bufferOffset);
    // Serialize message field [command]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.command, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AUVCommand
    let len;
    let data = new AUVCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [surge_speed]
    data.surge_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [command]
    data.command = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uuv_auv_control_allocator/AUVCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9d7c962f08b7f118399273df23351e7c';
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
    float64 surge_speed
    geometry_msgs/Wrench command
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
    MSG: geometry_msgs/Wrench
    # This represents force in free space, separated into
    # its linear and angular parts.
    Vector3  force
    Vector3  torque
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
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
    const resolved = new AUVCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.surge_speed !== undefined) {
      resolved.surge_speed = msg.surge_speed;
    }
    else {
      resolved.surge_speed = 0.0
    }

    if (msg.command !== undefined) {
      resolved.command = geometry_msgs.msg.Wrench.Resolve(msg.command)
    }
    else {
      resolved.command = new geometry_msgs.msg.Wrench()
    }

    return resolved;
    }
};

module.exports = AUVCommand;
