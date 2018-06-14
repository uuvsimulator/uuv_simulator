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

class SetUseGlobalCurrentVelRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.use_global = null;
    }
    else {
      if (initObj.hasOwnProperty('use_global')) {
        this.use_global = initObj.use_global
      }
      else {
        this.use_global = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetUseGlobalCurrentVelRequest
    // Serialize message field [use_global]
    bufferOffset = _serializer.bool(obj.use_global, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetUseGlobalCurrentVelRequest
    let len;
    let data = new SetUseGlobalCurrentVelRequest(null);
    // Deserialize message field [use_global]
    data.use_global = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_gazebo_ros_plugins_msgs/SetUseGlobalCurrentVelRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cb3581ad5adb4e1f612596312cf9e4fe';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    bool use_global
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetUseGlobalCurrentVelRequest(null);
    if (msg.use_global !== undefined) {
      resolved.use_global = msg.use_global;
    }
    else {
      resolved.use_global = false
    }

    return resolved;
    }
};

class SetUseGlobalCurrentVelResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetUseGlobalCurrentVelResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetUseGlobalCurrentVelResponse
    let len;
    let data = new SetUseGlobalCurrentVelResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_gazebo_ros_plugins_msgs/SetUseGlobalCurrentVelResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetUseGlobalCurrentVelResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SetUseGlobalCurrentVelRequest,
  Response: SetUseGlobalCurrentVelResponse,
  md5sum() { return '02d40f951486d0b4bee34e7b1c66f745'; },
  datatype() { return 'uuv_gazebo_ros_plugins_msgs/SetUseGlobalCurrentVel'; }
};
