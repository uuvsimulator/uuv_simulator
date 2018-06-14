// Auto-generated. Do not edit!

// (in-package uuv_world_ros_plugins_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetOriginSphericalCoordRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.latitude_deg = null;
      this.longitude_deg = null;
      this.altitude = null;
    }
    else {
      if (initObj.hasOwnProperty('latitude_deg')) {
        this.latitude_deg = initObj.latitude_deg
      }
      else {
        this.latitude_deg = 0.0;
      }
      if (initObj.hasOwnProperty('longitude_deg')) {
        this.longitude_deg = initObj.longitude_deg
      }
      else {
        this.longitude_deg = 0.0;
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetOriginSphericalCoordRequest
    // Serialize message field [latitude_deg]
    bufferOffset = _serializer.float64(obj.latitude_deg, buffer, bufferOffset);
    // Serialize message field [longitude_deg]
    bufferOffset = _serializer.float64(obj.longitude_deg, buffer, bufferOffset);
    // Serialize message field [altitude]
    bufferOffset = _serializer.float64(obj.altitude, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetOriginSphericalCoordRequest
    let len;
    let data = new SetOriginSphericalCoordRequest(null);
    // Deserialize message field [latitude_deg]
    data.latitude_deg = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [longitude_deg]
    data.longitude_deg = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [altitude]
    data.altitude = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_world_ros_plugins_msgs/SetOriginSphericalCoordRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '60457d630fe21cc5f8f6bd5d0fc90156';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    float64 latitude_deg
    
    float64 longitude_deg
    
    float64 altitude
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetOriginSphericalCoordRequest(null);
    if (msg.latitude_deg !== undefined) {
      resolved.latitude_deg = msg.latitude_deg;
    }
    else {
      resolved.latitude_deg = 0.0
    }

    if (msg.longitude_deg !== undefined) {
      resolved.longitude_deg = msg.longitude_deg;
    }
    else {
      resolved.longitude_deg = 0.0
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0.0
    }

    return resolved;
    }
};

class SetOriginSphericalCoordResponse {
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
    // Serializes a message object of type SetOriginSphericalCoordResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetOriginSphericalCoordResponse
    let len;
    let data = new SetOriginSphericalCoordResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_world_ros_plugins_msgs/SetOriginSphericalCoordResponse';
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
    const resolved = new SetOriginSphericalCoordResponse(null);
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
  Request: SetOriginSphericalCoordRequest,
  Response: SetOriginSphericalCoordResponse,
  md5sum() { return 'be1cd7093c79a14933c2ac116d54917a'; },
  datatype() { return 'uuv_world_ros_plugins_msgs/SetOriginSphericalCoord'; }
};
