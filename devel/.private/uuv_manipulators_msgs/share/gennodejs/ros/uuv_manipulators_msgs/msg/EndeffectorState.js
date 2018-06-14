// Auto-generated. Do not edit!

// (in-package uuv_manipulators_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class EndeffectorState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.state = null;
      this.position = null;
      this.effort = null;
    }
    else {
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = '';
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0.0;
      }
      if (initObj.hasOwnProperty('effort')) {
        this.effort = initObj.effort
      }
      else {
        this.effort = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EndeffectorState
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.string(obj.state, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = _serializer.float64(obj.position, buffer, bufferOffset);
    // Serialize message field [effort]
    bufferOffset = _serializer.float64(obj.effort, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EndeffectorState
    let len;
    let data = new EndeffectorState(null);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [effort]
    data.effort = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.state.length;
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uuv_manipulators_msgs/EndeffectorState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '696be13165827a929d588e22744f2679';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # States of the end-effector
    time      stamp
    string    state
    # Default states
    string    MOVING      = moving
    string    DISABLED    = disabled
    string    READY       = ready
    # Current state
    float64   position
    float64   effort
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EndeffectorState(null);
    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = ''
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0.0
    }

    if (msg.effort !== undefined) {
      resolved.effort = msg.effort;
    }
    else {
      resolved.effort = 0.0
    }

    return resolved;
    }
};

// Constants for message
EndeffectorState.Constants = {
  MOVING: 'moving',
  DISABLED: 'disabled',
  READY: 'ready',
}

module.exports = EndeffectorState;
