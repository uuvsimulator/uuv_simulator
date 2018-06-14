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

class EndeffectorCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command = null;
      this.ratio = null;
    }
    else {
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = '';
      }
      if (initObj.hasOwnProperty('ratio')) {
        this.ratio = initObj.ratio
      }
      else {
        this.ratio = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EndeffectorCommand
    // Serialize message field [command]
    bufferOffset = _serializer.string(obj.command, buffer, bufferOffset);
    // Serialize message field [ratio]
    bufferOffset = _serializer.float64(obj.ratio, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EndeffectorCommand
    let len;
    let data = new EndeffectorCommand(null);
    // Deserialize message field [command]
    data.command = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ratio]
    data.ratio = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.command.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uuv_manipulators_msgs/EndeffectorCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2949dea8a0d479d93952df57c48d98d6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Commands to the end-effector
    string  command     # Operation tag
    # Default commands
    string  EE_MOVE   = move
    string  EE_STOP   = stop
    # Place for arguments, if needed
    float64 ratio
    # Default ratios of aperture
    float64 EE_CLOSED = 0.0
    float64 EE_OPEN   = 100.0
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EndeffectorCommand(null);
    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = ''
    }

    if (msg.ratio !== undefined) {
      resolved.ratio = msg.ratio;
    }
    else {
      resolved.ratio = 0.0
    }

    return resolved;
    }
};

// Constants for message
EndeffectorCommand.Constants = {
  EE_MOVE: 'move',
  EE_STOP: 'stop',
  EE_CLOSED: 0.0,
  EE_OPEN: 100.0,
}

module.exports = EndeffectorCommand;
