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

//-----------------------------------------------------------

class PositionWithCovariance {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos = null;
      this.covariance = null;
    }
    else {
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('covariance')) {
        this.covariance = initObj.covariance
      }
      else {
        this.covariance = new Array(9).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PositionWithCovariance
    // Serialize message field [pos]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.pos, buffer, bufferOffset);
    // Check that the constant length array field [covariance] has the right length
    if (obj.covariance.length !== 9) {
      throw new Error('Unable to serialize array field covariance - length must be 9')
    }
    // Serialize message field [covariance]
    bufferOffset = _arraySerializer.float64(obj.covariance, buffer, bufferOffset, 9);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PositionWithCovariance
    let len;
    let data = new PositionWithCovariance(null);
    // Deserialize message field [pos]
    data.pos = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [covariance]
    data.covariance = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    return data;
  }

  static getMessageSize(object) {
    return 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uuv_sensor_plugins_ros_msgs/PositionWithCovariance';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4a54596b2ea1a0ed659f46ab0b26f202';
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
    
    # This represents a position in free space with uncertainty.
    
    geometry_msgs/Point pos
    
    # Row-major representation of the 3x3 covariance matrix
    float64[9] covariance
    
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
    const resolved = new PositionWithCovariance(null);
    if (msg.pos !== undefined) {
      resolved.pos = geometry_msgs.msg.Point.Resolve(msg.pos)
    }
    else {
      resolved.pos = new geometry_msgs.msg.Point()
    }

    if (msg.covariance !== undefined) {
      resolved.covariance = msg.covariance;
    }
    else {
      resolved.covariance = new Array(9).fill(0)
    }

    return resolved;
    }
};

module.exports = PositionWithCovariance;
