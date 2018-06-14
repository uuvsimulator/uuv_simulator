// Auto-generated. Do not edit!

// (in-package uuv_sensor_plugins_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Salinity {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.salinity = null;
      this.variance = null;
      this.unit = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('salinity')) {
        this.salinity = initObj.salinity
      }
      else {
        this.salinity = 0.0;
      }
      if (initObj.hasOwnProperty('variance')) {
        this.variance = initObj.variance
      }
      else {
        this.variance = 0.0;
      }
      if (initObj.hasOwnProperty('unit')) {
        this.unit = initObj.unit
      }
      else {
        this.unit = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Salinity
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [salinity]
    bufferOffset = _serializer.float64(obj.salinity, buffer, bufferOffset);
    // Serialize message field [variance]
    bufferOffset = _serializer.float64(obj.variance, buffer, bufferOffset);
    // Serialize message field [unit]
    bufferOffset = _serializer.string(obj.unit, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Salinity
    let len;
    let data = new Salinity(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [salinity]
    data.salinity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [variance]
    data.variance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [unit]
    data.unit = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.unit.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uuv_sensor_plugins_ros_msgs/Salinity';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4d20de37b8b3a344b3f4c36f2192b257';
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
    
    # Practical salinity units
    string PSU="psu"
    # Parts per million
    string PPM="ppm"
    # Parts per thousand
    string PPT="ppt"
    
    std_msgs/Header header
    float64 salinity
    float64 variance
    string unit
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Salinity(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.salinity !== undefined) {
      resolved.salinity = msg.salinity;
    }
    else {
      resolved.salinity = 0.0
    }

    if (msg.variance !== undefined) {
      resolved.variance = msg.variance;
    }
    else {
      resolved.variance = 0.0
    }

    if (msg.unit !== undefined) {
      resolved.unit = msg.unit;
    }
    else {
      resolved.unit = ''
    }

    return resolved;
    }
};

// Constants for message
Salinity.Constants = {
  PSU: '"psu"',
  PPM: '"ppm"',
  PPT: '"ppt"',
}

module.exports = Salinity;
