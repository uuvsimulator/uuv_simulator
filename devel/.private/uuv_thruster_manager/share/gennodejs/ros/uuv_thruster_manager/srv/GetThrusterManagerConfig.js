// Auto-generated. Do not edit!

// (in-package uuv_thruster_manager.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetThrusterManagerConfigRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetThrusterManagerConfigRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetThrusterManagerConfigRequest
    let len;
    let data = new GetThrusterManagerConfigRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_thruster_manager/GetThrusterManagerConfigRequest';
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
    const resolved = new GetThrusterManagerConfigRequest(null);
    return resolved;
    }
};

class GetThrusterManagerConfigResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tf_prefix = null;
      this.base_link = null;
      this.thruster_frame_base = null;
      this.thruster_topic_prefix = null;
      this.thruster_topic_suffix = null;
      this.timeout = null;
      this.max_thrust = null;
      this.n_thrusters = null;
      this.allocation_matrix = null;
    }
    else {
      if (initObj.hasOwnProperty('tf_prefix')) {
        this.tf_prefix = initObj.tf_prefix
      }
      else {
        this.tf_prefix = '';
      }
      if (initObj.hasOwnProperty('base_link')) {
        this.base_link = initObj.base_link
      }
      else {
        this.base_link = '';
      }
      if (initObj.hasOwnProperty('thruster_frame_base')) {
        this.thruster_frame_base = initObj.thruster_frame_base
      }
      else {
        this.thruster_frame_base = '';
      }
      if (initObj.hasOwnProperty('thruster_topic_prefix')) {
        this.thruster_topic_prefix = initObj.thruster_topic_prefix
      }
      else {
        this.thruster_topic_prefix = '';
      }
      if (initObj.hasOwnProperty('thruster_topic_suffix')) {
        this.thruster_topic_suffix = initObj.thruster_topic_suffix
      }
      else {
        this.thruster_topic_suffix = '';
      }
      if (initObj.hasOwnProperty('timeout')) {
        this.timeout = initObj.timeout
      }
      else {
        this.timeout = 0.0;
      }
      if (initObj.hasOwnProperty('max_thrust')) {
        this.max_thrust = initObj.max_thrust
      }
      else {
        this.max_thrust = 0.0;
      }
      if (initObj.hasOwnProperty('n_thrusters')) {
        this.n_thrusters = initObj.n_thrusters
      }
      else {
        this.n_thrusters = 0;
      }
      if (initObj.hasOwnProperty('allocation_matrix')) {
        this.allocation_matrix = initObj.allocation_matrix
      }
      else {
        this.allocation_matrix = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetThrusterManagerConfigResponse
    // Serialize message field [tf_prefix]
    bufferOffset = _serializer.string(obj.tf_prefix, buffer, bufferOffset);
    // Serialize message field [base_link]
    bufferOffset = _serializer.string(obj.base_link, buffer, bufferOffset);
    // Serialize message field [thruster_frame_base]
    bufferOffset = _serializer.string(obj.thruster_frame_base, buffer, bufferOffset);
    // Serialize message field [thruster_topic_prefix]
    bufferOffset = _serializer.string(obj.thruster_topic_prefix, buffer, bufferOffset);
    // Serialize message field [thruster_topic_suffix]
    bufferOffset = _serializer.string(obj.thruster_topic_suffix, buffer, bufferOffset);
    // Serialize message field [timeout]
    bufferOffset = _serializer.float64(obj.timeout, buffer, bufferOffset);
    // Serialize message field [max_thrust]
    bufferOffset = _serializer.float64(obj.max_thrust, buffer, bufferOffset);
    // Serialize message field [n_thrusters]
    bufferOffset = _serializer.int32(obj.n_thrusters, buffer, bufferOffset);
    // Serialize message field [allocation_matrix]
    bufferOffset = _arraySerializer.float64(obj.allocation_matrix, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetThrusterManagerConfigResponse
    let len;
    let data = new GetThrusterManagerConfigResponse(null);
    // Deserialize message field [tf_prefix]
    data.tf_prefix = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [base_link]
    data.base_link = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [thruster_frame_base]
    data.thruster_frame_base = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [thruster_topic_prefix]
    data.thruster_topic_prefix = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [thruster_topic_suffix]
    data.thruster_topic_suffix = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [timeout]
    data.timeout = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_thrust]
    data.max_thrust = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [n_thrusters]
    data.n_thrusters = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [allocation_matrix]
    data.allocation_matrix = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.tf_prefix.length;
    length += object.base_link.length;
    length += object.thruster_frame_base.length;
    length += object.thruster_topic_prefix.length;
    length += object.thruster_topic_suffix.length;
    length += 8 * object.allocation_matrix.length;
    return length + 44;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_thruster_manager/GetThrusterManagerConfigResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b5a2d9d3bb510dd91fdb03f95e95b8de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string tf_prefix
    string base_link
    string thruster_frame_base
    string thruster_topic_prefix
    string thruster_topic_suffix
    float64 timeout
    float64 max_thrust
    int32 n_thrusters
    float64[] allocation_matrix
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetThrusterManagerConfigResponse(null);
    if (msg.tf_prefix !== undefined) {
      resolved.tf_prefix = msg.tf_prefix;
    }
    else {
      resolved.tf_prefix = ''
    }

    if (msg.base_link !== undefined) {
      resolved.base_link = msg.base_link;
    }
    else {
      resolved.base_link = ''
    }

    if (msg.thruster_frame_base !== undefined) {
      resolved.thruster_frame_base = msg.thruster_frame_base;
    }
    else {
      resolved.thruster_frame_base = ''
    }

    if (msg.thruster_topic_prefix !== undefined) {
      resolved.thruster_topic_prefix = msg.thruster_topic_prefix;
    }
    else {
      resolved.thruster_topic_prefix = ''
    }

    if (msg.thruster_topic_suffix !== undefined) {
      resolved.thruster_topic_suffix = msg.thruster_topic_suffix;
    }
    else {
      resolved.thruster_topic_suffix = ''
    }

    if (msg.timeout !== undefined) {
      resolved.timeout = msg.timeout;
    }
    else {
      resolved.timeout = 0.0
    }

    if (msg.max_thrust !== undefined) {
      resolved.max_thrust = msg.max_thrust;
    }
    else {
      resolved.max_thrust = 0.0
    }

    if (msg.n_thrusters !== undefined) {
      resolved.n_thrusters = msg.n_thrusters;
    }
    else {
      resolved.n_thrusters = 0
    }

    if (msg.allocation_matrix !== undefined) {
      resolved.allocation_matrix = msg.allocation_matrix;
    }
    else {
      resolved.allocation_matrix = []
    }

    return resolved;
    }
};

module.exports = {
  Request: GetThrusterManagerConfigRequest,
  Response: GetThrusterManagerConfigResponse,
  md5sum() { return 'b5a2d9d3bb510dd91fdb03f95e95b8de'; },
  datatype() { return 'uuv_thruster_manager/GetThrusterManagerConfig'; }
};
