// Auto-generated. Do not edit!

// (in-package uuv_gazebo_ros_plugins_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let UnderwaterObjectModel = require('../msg/UnderwaterObjectModel.js');

//-----------------------------------------------------------

class GetModelPropertiesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetModelPropertiesRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetModelPropertiesRequest
    let len;
    let data = new GetModelPropertiesRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_gazebo_ros_plugins_msgs/GetModelPropertiesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetModelPropertiesRequest(null);
    return resolved;
    }
};

class GetModelPropertiesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.link_names = null;
      this.models = null;
    }
    else {
      if (initObj.hasOwnProperty('link_names')) {
        this.link_names = initObj.link_names
      }
      else {
        this.link_names = [];
      }
      if (initObj.hasOwnProperty('models')) {
        this.models = initObj.models
      }
      else {
        this.models = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetModelPropertiesResponse
    // Serialize message field [link_names]
    bufferOffset = _arraySerializer.string(obj.link_names, buffer, bufferOffset, null);
    // Serialize message field [models]
    // Serialize the length for message field [models]
    bufferOffset = _serializer.uint32(obj.models.length, buffer, bufferOffset);
    obj.models.forEach((val) => {
      bufferOffset = UnderwaterObjectModel.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetModelPropertiesResponse
    let len;
    let data = new GetModelPropertiesResponse(null);
    // Deserialize message field [link_names]
    data.link_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [models]
    // Deserialize array length for message field [models]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.models = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.models[i] = UnderwaterObjectModel.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.link_names.forEach((val) => {
      length += 4 + val.length;
    });
    object.models.forEach((val) => {
      length += UnderwaterObjectModel.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_gazebo_ros_plugins_msgs/GetModelPropertiesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '222d64ab6fa46c24f1abd065170ebe7a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[]  link_names
    uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel[] models
    
    
    ================================================================================
    MSG: uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel
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
    const resolved = new GetModelPropertiesResponse(null);
    if (msg.link_names !== undefined) {
      resolved.link_names = msg.link_names;
    }
    else {
      resolved.link_names = []
    }

    if (msg.models !== undefined) {
      resolved.models = new Array(msg.models.length);
      for (let i = 0; i < resolved.models.length; ++i) {
        resolved.models[i] = UnderwaterObjectModel.Resolve(msg.models[i]);
      }
    }
    else {
      resolved.models = []
    }

    return resolved;
    }
};

module.exports = {
  Request: GetModelPropertiesRequest,
  Response: GetModelPropertiesResponse,
  md5sum() { return '222d64ab6fa46c24f1abd065170ebe7a'; },
  datatype() { return 'uuv_gazebo_ros_plugins_msgs/GetModelProperties'; }
};
