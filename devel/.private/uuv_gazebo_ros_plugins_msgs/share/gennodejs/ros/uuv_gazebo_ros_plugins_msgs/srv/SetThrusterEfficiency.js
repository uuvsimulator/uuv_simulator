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

class SetThrusterEfficiencyRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.efficiency = null;
    }
    else {
      if (initObj.hasOwnProperty('efficiency')) {
        this.efficiency = initObj.efficiency
      }
      else {
        this.efficiency = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetThrusterEfficiencyRequest
    // Serialize message field [efficiency]
    bufferOffset = _serializer.float64(obj.efficiency, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetThrusterEfficiencyRequest
    let len;
    let data = new SetThrusterEfficiencyRequest(null);
    // Deserialize message field [efficiency]
    data.efficiency = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_gazebo_ros_plugins_msgs/SetThrusterEfficiencyRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b80ec71e671b93e4cc403df1ac4c8a86';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    float64 efficiency
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetThrusterEfficiencyRequest(null);
    if (msg.efficiency !== undefined) {
      resolved.efficiency = msg.efficiency;
    }
    else {
      resolved.efficiency = 0.0
    }

    return resolved;
    }
};

class SetThrusterEfficiencyResponse {
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
    // Serializes a message object of type SetThrusterEfficiencyResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetThrusterEfficiencyResponse
    let len;
    let data = new SetThrusterEfficiencyResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_gazebo_ros_plugins_msgs/SetThrusterEfficiencyResponse';
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
    const resolved = new SetThrusterEfficiencyResponse(null);
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
  Request: SetThrusterEfficiencyRequest,
  Response: SetThrusterEfficiencyResponse,
  md5sum() { return '60f827235457ddfd6a19b596030b49ad'; },
  datatype() { return 'uuv_gazebo_ros_plugins_msgs/SetThrusterEfficiency'; }
};
