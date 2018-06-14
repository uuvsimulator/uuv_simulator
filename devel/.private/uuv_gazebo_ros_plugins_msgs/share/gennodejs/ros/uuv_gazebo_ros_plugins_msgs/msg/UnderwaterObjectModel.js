// Auto-generated. Do not edit!

// (in-package uuv_gazebo_ros_plugins_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class UnderwaterObjectModel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.added_mass = null;
      this.linear_damping = null;
      this.linear_damping_forward_speed = null;
      this.quadratic_damping = null;
      this.volume = null;
      this.bbox_height = null;
      this.bbox_length = null;
      this.bbox_width = null;
      this.fluid_density = null;
      this.cob = null;
      this.neutrally_buoyant = null;
      this.inertia = null;
    }
    else {
      if (initObj.hasOwnProperty('added_mass')) {
        this.added_mass = initObj.added_mass
      }
      else {
        this.added_mass = [];
      }
      if (initObj.hasOwnProperty('linear_damping')) {
        this.linear_damping = initObj.linear_damping
      }
      else {
        this.linear_damping = [];
      }
      if (initObj.hasOwnProperty('linear_damping_forward_speed')) {
        this.linear_damping_forward_speed = initObj.linear_damping_forward_speed
      }
      else {
        this.linear_damping_forward_speed = [];
      }
      if (initObj.hasOwnProperty('quadratic_damping')) {
        this.quadratic_damping = initObj.quadratic_damping
      }
      else {
        this.quadratic_damping = [];
      }
      if (initObj.hasOwnProperty('volume')) {
        this.volume = initObj.volume
      }
      else {
        this.volume = 0.0;
      }
      if (initObj.hasOwnProperty('bbox_height')) {
        this.bbox_height = initObj.bbox_height
      }
      else {
        this.bbox_height = 0.0;
      }
      if (initObj.hasOwnProperty('bbox_length')) {
        this.bbox_length = initObj.bbox_length
      }
      else {
        this.bbox_length = 0.0;
      }
      if (initObj.hasOwnProperty('bbox_width')) {
        this.bbox_width = initObj.bbox_width
      }
      else {
        this.bbox_width = 0.0;
      }
      if (initObj.hasOwnProperty('fluid_density')) {
        this.fluid_density = initObj.fluid_density
      }
      else {
        this.fluid_density = 0.0;
      }
      if (initObj.hasOwnProperty('cob')) {
        this.cob = initObj.cob
      }
      else {
        this.cob = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('neutrally_buoyant')) {
        this.neutrally_buoyant = initObj.neutrally_buoyant
      }
      else {
        this.neutrally_buoyant = false;
      }
      if (initObj.hasOwnProperty('inertia')) {
        this.inertia = initObj.inertia
      }
      else {
        this.inertia = new geometry_msgs.msg.Inertia();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UnderwaterObjectModel
    // Serialize message field [added_mass]
    bufferOffset = _arraySerializer.float64(obj.added_mass, buffer, bufferOffset, null);
    // Serialize message field [linear_damping]
    bufferOffset = _arraySerializer.float64(obj.linear_damping, buffer, bufferOffset, null);
    // Serialize message field [linear_damping_forward_speed]
    bufferOffset = _arraySerializer.float64(obj.linear_damping_forward_speed, buffer, bufferOffset, null);
    // Serialize message field [quadratic_damping]
    bufferOffset = _arraySerializer.float64(obj.quadratic_damping, buffer, bufferOffset, null);
    // Serialize message field [volume]
    bufferOffset = _serializer.float64(obj.volume, buffer, bufferOffset);
    // Serialize message field [bbox_height]
    bufferOffset = _serializer.float64(obj.bbox_height, buffer, bufferOffset);
    // Serialize message field [bbox_length]
    bufferOffset = _serializer.float64(obj.bbox_length, buffer, bufferOffset);
    // Serialize message field [bbox_width]
    bufferOffset = _serializer.float64(obj.bbox_width, buffer, bufferOffset);
    // Serialize message field [fluid_density]
    bufferOffset = _serializer.float64(obj.fluid_density, buffer, bufferOffset);
    // Serialize message field [cob]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.cob, buffer, bufferOffset);
    // Serialize message field [neutrally_buoyant]
    bufferOffset = _serializer.bool(obj.neutrally_buoyant, buffer, bufferOffset);
    // Serialize message field [inertia]
    bufferOffset = geometry_msgs.msg.Inertia.serialize(obj.inertia, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UnderwaterObjectModel
    let len;
    let data = new UnderwaterObjectModel(null);
    // Deserialize message field [added_mass]
    data.added_mass = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [linear_damping]
    data.linear_damping = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [linear_damping_forward_speed]
    data.linear_damping_forward_speed = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [quadratic_damping]
    data.quadratic_damping = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [volume]
    data.volume = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bbox_height]
    data.bbox_height = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bbox_length]
    data.bbox_length = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bbox_width]
    data.bbox_width = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [fluid_density]
    data.fluid_density = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cob]
    data.cob = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [neutrally_buoyant]
    data.neutrally_buoyant = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [inertia]
    data.inertia = geometry_msgs.msg.Inertia.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.added_mass.length;
    length += 8 * object.linear_damping.length;
    length += 8 * object.linear_damping_forward_speed.length;
    length += 8 * object.quadratic_damping.length;
    return length + 161;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '35ada57addb7202af96020ea2ddc109c';
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
    
    float64[] added_mass
    float64[] linear_damping
    float64[] linear_damping_forward_speed
    float64[] quadratic_damping
    float64 volume
    float64 bbox_height
    float64 bbox_length
    float64 bbox_width
    float64 fluid_density
    geometry_msgs/Vector3 cob
    bool neutrally_buoyant
    geometry_msgs/Inertia inertia
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
    MSG: geometry_msgs/Inertia
    # Mass [kg]
    float64 m
    
    # Center of mass [m]
    geometry_msgs/Vector3 com
    
    # Inertia Tensor [kg-m^2]
    #     | ixx ixy ixz |
    # I = | ixy iyy iyz |
    #     | ixz iyz izz |
    float64 ixx
    float64 ixy
    float64 ixz
    float64 iyy
    float64 iyz
    float64 izz
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UnderwaterObjectModel(null);
    if (msg.added_mass !== undefined) {
      resolved.added_mass = msg.added_mass;
    }
    else {
      resolved.added_mass = []
    }

    if (msg.linear_damping !== undefined) {
      resolved.linear_damping = msg.linear_damping;
    }
    else {
      resolved.linear_damping = []
    }

    if (msg.linear_damping_forward_speed !== undefined) {
      resolved.linear_damping_forward_speed = msg.linear_damping_forward_speed;
    }
    else {
      resolved.linear_damping_forward_speed = []
    }

    if (msg.quadratic_damping !== undefined) {
      resolved.quadratic_damping = msg.quadratic_damping;
    }
    else {
      resolved.quadratic_damping = []
    }

    if (msg.volume !== undefined) {
      resolved.volume = msg.volume;
    }
    else {
      resolved.volume = 0.0
    }

    if (msg.bbox_height !== undefined) {
      resolved.bbox_height = msg.bbox_height;
    }
    else {
      resolved.bbox_height = 0.0
    }

    if (msg.bbox_length !== undefined) {
      resolved.bbox_length = msg.bbox_length;
    }
    else {
      resolved.bbox_length = 0.0
    }

    if (msg.bbox_width !== undefined) {
      resolved.bbox_width = msg.bbox_width;
    }
    else {
      resolved.bbox_width = 0.0
    }

    if (msg.fluid_density !== undefined) {
      resolved.fluid_density = msg.fluid_density;
    }
    else {
      resolved.fluid_density = 0.0
    }

    if (msg.cob !== undefined) {
      resolved.cob = geometry_msgs.msg.Vector3.Resolve(msg.cob)
    }
    else {
      resolved.cob = new geometry_msgs.msg.Vector3()
    }

    if (msg.neutrally_buoyant !== undefined) {
      resolved.neutrally_buoyant = msg.neutrally_buoyant;
    }
    else {
      resolved.neutrally_buoyant = false
    }

    if (msg.inertia !== undefined) {
      resolved.inertia = geometry_msgs.msg.Inertia.Resolve(msg.inertia)
    }
    else {
      resolved.inertia = new geometry_msgs.msg.Inertia()
    }

    return resolved;
    }
};

module.exports = UnderwaterObjectModel;
