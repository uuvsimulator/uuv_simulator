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

class ArmConfigCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command = null;
      this.args = null;
    }
    else {
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = '';
      }
      if (initObj.hasOwnProperty('args')) {
        this.args = initObj.args
      }
      else {
        this.args = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmConfigCommand
    // Serialize message field [command]
    bufferOffset = _serializer.string(obj.command, buffer, bufferOffset);
    // Serialize message field [args]
    bufferOffset = _serializer.string(obj.args, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmConfigCommand
    let len;
    let data = new ArmConfigCommand(null);
    // Deserialize message field [command]
    data.command = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [args]
    data.args = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.command.length;
    length += object.args.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uuv_manipulators_msgs/ArmConfigCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '428ecc7602e5c382dfc52081cc34f5a6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Commands to drive the arm to default configuration
    string  command     # Operation tag
    # Default commands
    string  HOME      = home
    # Place for arguments, if needed
    string  args
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmConfigCommand(null);
    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = ''
    }

    if (msg.args !== undefined) {
      resolved.args = msg.args;
    }
    else {
      resolved.args = ''
    }

    return resolved;
    }
};

// Constants for message
ArmConfigCommand.Constants = {
  HOME: 'home',
}

module.exports = ArmConfigCommand;
