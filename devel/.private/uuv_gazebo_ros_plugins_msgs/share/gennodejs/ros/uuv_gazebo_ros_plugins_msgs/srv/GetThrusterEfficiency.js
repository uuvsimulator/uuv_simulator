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

class GetThrusterEfficiencyRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetThrusterEfficiencyRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetThrusterEfficiencyRequest
    let len;
    let data = new GetThrusterEfficiencyRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_gazebo_ros_plugins_msgs/GetThrusterEfficiencyRequest';
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
    const resolved = new GetThrusterEfficiencyRequest(null);
    return resolved;
    }
};

class GetThrusterEfficiencyResponse {
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
    // Serializes a message object of type GetThrusterEfficiencyResponse
    // Serialize message field [efficiency]
    bufferOffset = _serializer.float64(obj.efficiency, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetThrusterEfficiencyResponse
    let len;
    let data = new GetThrusterEfficiencyResponse(null);
    // Deserialize message field [efficiency]
    data.efficiency = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_gazebo_ros_plugins_msgs/GetThrusterEfficiencyResponse';
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
    const resolved = new GetThrusterEfficiencyResponse(null);
    if (msg.efficiency !== undefined) {
      resolved.efficiency = msg.efficiency;
    }
    else {
      resolved.efficiency = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetThrusterEfficiencyRequest,
  Response: GetThrusterEfficiencyResponse,
  md5sum() { return 'b80ec71e671b93e4cc403df1ac4c8a86'; },
  datatype() { return 'uuv_gazebo_ros_plugins_msgs/GetThrusterEfficiency'; }
};
