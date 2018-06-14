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

class GetThrusterCurveRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.min = null;
      this.max = null;
      this.n_points = null;
    }
    else {
      if (initObj.hasOwnProperty('min')) {
        this.min = initObj.min
      }
      else {
        this.min = 0.0;
      }
      if (initObj.hasOwnProperty('max')) {
        this.max = initObj.max
      }
      else {
        this.max = 0.0;
      }
      if (initObj.hasOwnProperty('n_points')) {
        this.n_points = initObj.n_points
      }
      else {
        this.n_points = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetThrusterCurveRequest
    // Serialize message field [min]
    bufferOffset = _serializer.float64(obj.min, buffer, bufferOffset);
    // Serialize message field [max]
    bufferOffset = _serializer.float64(obj.max, buffer, bufferOffset);
    // Serialize message field [n_points]
    bufferOffset = _serializer.int32(obj.n_points, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetThrusterCurveRequest
    let len;
    let data = new GetThrusterCurveRequest(null);
    // Deserialize message field [min]
    data.min = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max]
    data.max = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [n_points]
    data.n_points = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_thruster_manager/GetThrusterCurveRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3b8a1df953063c9de5f9a6419991fa0c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    float64 min
    float64 max
    int32 n_points
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetThrusterCurveRequest(null);
    if (msg.min !== undefined) {
      resolved.min = msg.min;
    }
    else {
      resolved.min = 0.0
    }

    if (msg.max !== undefined) {
      resolved.max = msg.max;
    }
    else {
      resolved.max = 0.0
    }

    if (msg.n_points !== undefined) {
      resolved.n_points = msg.n_points;
    }
    else {
      resolved.n_points = 0
    }

    return resolved;
    }
};

class GetThrusterCurveResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input = null;
      this.thrust = null;
    }
    else {
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = [];
      }
      if (initObj.hasOwnProperty('thrust')) {
        this.thrust = initObj.thrust
      }
      else {
        this.thrust = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetThrusterCurveResponse
    // Serialize message field [input]
    bufferOffset = _arraySerializer.float64(obj.input, buffer, bufferOffset, null);
    // Serialize message field [thrust]
    bufferOffset = _arraySerializer.float64(obj.thrust, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetThrusterCurveResponse
    let len;
    let data = new GetThrusterCurveResponse(null);
    // Deserialize message field [input]
    data.input = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [thrust]
    data.thrust = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.input.length;
    length += 8 * object.thrust.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_thruster_manager/GetThrusterCurveResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '183802edaba8fb9ba8a2d917792277f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] input
    float64[] thrust
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetThrusterCurveResponse(null);
    if (msg.input !== undefined) {
      resolved.input = msg.input;
    }
    else {
      resolved.input = []
    }

    if (msg.thrust !== undefined) {
      resolved.thrust = msg.thrust;
    }
    else {
      resolved.thrust = []
    }

    return resolved;
    }
};

module.exports = {
  Request: GetThrusterCurveRequest,
  Response: GetThrusterCurveResponse,
  md5sum() { return '93e0d8a78977b3619bdc09f290ef57fa'; },
  datatype() { return 'uuv_thruster_manager/GetThrusterCurve'; }
};
