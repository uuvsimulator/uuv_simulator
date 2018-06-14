// Auto-generated. Do not edit!

// (in-package uuv_sensor_plugins_ros_msgs.msg)


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

class ChemicalParticleConcentration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.position = null;
      this.latitude = null;
      this.longitude = null;
      this.depth = null;
      this.concentration = null;
      this.is_measuring = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('depth')) {
        this.depth = initObj.depth
      }
      else {
        this.depth = 0.0;
      }
      if (initObj.hasOwnProperty('concentration')) {
        this.concentration = initObj.concentration
      }
      else {
        this.concentration = 0.0;
      }
      if (initObj.hasOwnProperty('is_measuring')) {
        this.is_measuring = initObj.is_measuring
      }
      else {
        this.is_measuring = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ChemicalParticleConcentration
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    // Serialize message field [depth]
    bufferOffset = _serializer.float64(obj.depth, buffer, bufferOffset);
    // Serialize message field [concentration]
    bufferOffset = _serializer.float64(obj.concentration, buffer, bufferOffset);
    // Serialize message field [is_measuring]
    bufferOffset = _serializer.bool(obj.is_measuring, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ChemicalParticleConcentration
    let len;
    let data = new ChemicalParticleConcentration(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [depth]
    data.depth = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [concentration]
    data.concentration = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [is_measuring]
    data.is_measuring = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 57;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uuv_sensor_plugins_ros_msgs/ChemicalParticleConcentration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f17ac73d7dd3916a4fd6f721f0ccbe37';
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
    
    Header header
    # Sensor link position
    geometry_msgs/Vector3 position
    # Latitude in degrees (WGS84 coordinates)
    float64 latitude
    # Longiture in degrees (WGS84 coordinates)
    float64 longitude
    # Depth in meters (WGS84 coordinates)
    float64 depth
    # Particle concentration
    float64 concentration
    # Status, false if no plume particles are received
    bool is_measuring
    
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
    const resolved = new ChemicalParticleConcentration(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Vector3.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Vector3()
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.depth !== undefined) {
      resolved.depth = msg.depth;
    }
    else {
      resolved.depth = 0.0
    }

    if (msg.concentration !== undefined) {
      resolved.concentration = msg.concentration;
    }
    else {
      resolved.concentration = 0.0
    }

    if (msg.is_measuring !== undefined) {
      resolved.is_measuring = msg.is_measuring;
    }
    else {
      resolved.is_measuring = false
    }

    return resolved;
    }
};

module.exports = ChemicalParticleConcentration;
