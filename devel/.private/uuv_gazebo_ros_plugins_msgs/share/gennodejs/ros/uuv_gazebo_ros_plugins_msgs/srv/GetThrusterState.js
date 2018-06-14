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


//-----------------------------------------------------------

class GetThrusterStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetThrusterStateRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetThrusterStateRequest
    let len;
    let data = new GetThrusterStateRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_gazebo_ros_plugins_msgs/GetThrusterStateRequest';
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
    const resolved = new GetThrusterStateRequest(null);
    return resolved;
    }
};

class GetThrusterStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.is_on = null;
    }
    else {
      if (initObj.hasOwnProperty('is_on')) {
        this.is_on = initObj.is_on
      }
      else {
        this.is_on = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetThrusterStateResponse
    // Serialize message field [is_on]
    bufferOffset = _serializer.bool(obj.is_on, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetThrusterStateResponse
    let len;
    let data = new GetThrusterStateResponse(null);
    // Deserialize message field [is_on]
    data.is_on = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_gazebo_ros_plugins_msgs/GetThrusterStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e2fdda8431274beee70eebaa081c813e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool is_on
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetThrusterStateResponse(null);
    if (msg.is_on !== undefined) {
      resolved.is_on = msg.is_on;
    }
    else {
      resolved.is_on = false
    }

    return resolved;
    }
};

module.exports = {
  Request: GetThrusterStateRequest,
  Response: GetThrusterStateResponse,
  md5sum() { return 'e2fdda8431274beee70eebaa081c813e'; },
  datatype() { return 'uuv_gazebo_ros_plugins_msgs/GetThrusterState'; }
};
