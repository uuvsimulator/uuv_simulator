// Auto-generated. Do not edit!

// (in-package uuv_sensor_plugins_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DVLBeam = require('./DVLBeam.js');
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DVL {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.velocity = null;
      this.velocity_covariance = null;
      this.altitude = null;
      this.beams = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('velocity_covariance')) {
        this.velocity_covariance = initObj.velocity_covariance
      }
      else {
        this.velocity_covariance = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0.0;
      }
      if (initObj.hasOwnProperty('beams')) {
        this.beams = initObj.beams
      }
      else {
        this.beams = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DVL
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity, buffer, bufferOffset);
    // Check that the constant length array field [velocity_covariance] has the right length
    if (obj.velocity_covariance.length !== 9) {
      throw new Error('Unable to serialize array field velocity_covariance - length must be 9')
    }
    // Serialize message field [velocity_covariance]
    bufferOffset = _arraySerializer.float64(obj.velocity_covariance, buffer, bufferOffset, 9);
    // Serialize message field [altitude]
    bufferOffset = _serializer.float64(obj.altitude, buffer, bufferOffset);
    // Serialize message field [beams]
    // Serialize the length for message field [beams]
    bufferOffset = _serializer.uint32(obj.beams.length, buffer, bufferOffset);
    obj.beams.forEach((val) => {
      bufferOffset = DVLBeam.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DVL
    let len;
    let data = new DVL(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity_covariance]
    data.velocity_covariance = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [altitude]
    data.altitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [beams]
    // Deserialize array length for message field [beams]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.beams = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.beams[i] = DVLBeam.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.beams.forEach((val) => {
      length += DVLBeam.getMessageSize(val);
    });
    return length + 108;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uuv_sensor_plugins_ros_msgs/DVL';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '02bba6182b6f271447d7f88473256572';
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
    
    # This is a message to hold data from a DVL sensor (Doppler Velocity Log).
    #
    # Distances are in [m], velocities in [m/s]
    #
    # If the covariance is known, it should be filled.
    # If it is unknown, it should be set to all zeros.
    # If a measurement was invalid, its covariance should be set to -1 so it can be
    # disregarded.
    #
    # DVLBeams are optional. If they are set they contain individual ranges and 1D
    # doppler velocity estimates orthogonal to the ray.
    
    Header header
    # Measured velocity [m/s]
    geometry_msgs/Vector3 velocity
    # Row major, xyz axes
    float64[9] velocity_covariance
    # Altitude of the vehicle
    float64 altitude
    DVLBeam[] beams
    
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
    ================================================================================
    MSG: uuv_sensor_plugins_ros_msgs/DVLBeam
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
    
    # measured range [m] or < 0 if invalid
    float64 range
    float64 range_covariance
    
    float64 velocity # measured velocity [m/s] of corr. beam
    float64 velocity_covariance
    
    # Beam link pose wrt DVL link frame
    geometry_msgs/PoseStamped pose
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DVL(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = geometry_msgs.msg.Vector3.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.velocity_covariance !== undefined) {
      resolved.velocity_covariance = msg.velocity_covariance;
    }
    else {
      resolved.velocity_covariance = new Array(9).fill(0)
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0.0
    }

    if (msg.beams !== undefined) {
      resolved.beams = new Array(msg.beams.length);
      for (let i = 0; i < resolved.beams.length; ++i) {
        resolved.beams[i] = DVLBeam.Resolve(msg.beams[i]);
      }
    }
    else {
      resolved.beams = []
    }

    return resolved;
    }
};

module.exports = DVL;
