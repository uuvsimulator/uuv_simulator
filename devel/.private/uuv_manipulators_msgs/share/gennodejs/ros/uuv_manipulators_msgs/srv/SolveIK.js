// Auto-generated. Do not edit!

// (in-package uuv_manipulators_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SolveIKRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose = null;
      this.seed_angles = null;
    }
    else {
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('seed_angles')) {
        this.seed_angles = initObj.seed_angles
      }
      else {
        this.seed_angles = new sensor_msgs.msg.JointState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SolveIKRequest
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [seed_angles]
    bufferOffset = sensor_msgs.msg.JointState.serialize(obj.seed_angles, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SolveIKRequest
    let len;
    let data = new SolveIKRequest(null);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [seed_angles]
    data.seed_angles = sensor_msgs.msg.JointState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.JointState.getMessageSize(object.seed_angles);
    return length + 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_manipulators_msgs/SolveIKRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b6ee325190e9b15bbcd7df5e003c1fe1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose pose
    sensor_msgs/JointState seed_angles
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: sensor_msgs/JointState
    # This is a message that holds data to describe the state of a set of torque controlled joints. 
    #
    # The state of each joint (revolute or prismatic) is defined by:
    #  * the position of the joint (rad or m),
    #  * the velocity of the joint (rad/s or m/s) and 
    #  * the effort that is applied in the joint (Nm or N).
    #
    # Each joint is uniquely identified by its name
    # The header specifies the time at which the joint states were recorded. All the joint states
    # in one message have to be recorded at the same time.
    #
    # This message consists of a multiple arrays, one for each part of the joint state. 
    # The goal is to make each of the fields optional. When e.g. your joints have no
    # effort associated with them, you can leave the effort array empty. 
    #
    # All arrays in this message should have the same size, or be empty.
    # This is the only way to uniquely associate the joint name with the correct
    # states.
    
    
    Header header
    
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SolveIKRequest(null);
    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.seed_angles !== undefined) {
      resolved.seed_angles = sensor_msgs.msg.JointState.Resolve(msg.seed_angles)
    }
    else {
      resolved.seed_angles = new sensor_msgs.msg.JointState()
    }

    return resolved;
    }
};

class SolveIKResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joints = null;
      this.isValid = null;
    }
    else {
      if (initObj.hasOwnProperty('joints')) {
        this.joints = initObj.joints
      }
      else {
        this.joints = new sensor_msgs.msg.JointState();
      }
      if (initObj.hasOwnProperty('isValid')) {
        this.isValid = initObj.isValid
      }
      else {
        this.isValid = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SolveIKResponse
    // Serialize message field [joints]
    bufferOffset = sensor_msgs.msg.JointState.serialize(obj.joints, buffer, bufferOffset);
    // Serialize message field [isValid]
    bufferOffset = _serializer.bool(obj.isValid, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SolveIKResponse
    let len;
    let data = new SolveIKResponse(null);
    // Deserialize message field [joints]
    data.joints = sensor_msgs.msg.JointState.deserialize(buffer, bufferOffset);
    // Deserialize message field [isValid]
    data.isValid = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.JointState.getMessageSize(object.joints);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_manipulators_msgs/SolveIKResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a57abfd77e0408ae11a6fe4dd5adf1fc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/JointState joints
    bool isValid
    
    
    ================================================================================
    MSG: sensor_msgs/JointState
    # This is a message that holds data to describe the state of a set of torque controlled joints. 
    #
    # The state of each joint (revolute or prismatic) is defined by:
    #  * the position of the joint (rad or m),
    #  * the velocity of the joint (rad/s or m/s) and 
    #  * the effort that is applied in the joint (Nm or N).
    #
    # Each joint is uniquely identified by its name
    # The header specifies the time at which the joint states were recorded. All the joint states
    # in one message have to be recorded at the same time.
    #
    # This message consists of a multiple arrays, one for each part of the joint state. 
    # The goal is to make each of the fields optional. When e.g. your joints have no
    # effort associated with them, you can leave the effort array empty. 
    #
    # All arrays in this message should have the same size, or be empty.
    # This is the only way to uniquely associate the joint name with the correct
    # states.
    
    
    Header header
    
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SolveIKResponse(null);
    if (msg.joints !== undefined) {
      resolved.joints = sensor_msgs.msg.JointState.Resolve(msg.joints)
    }
    else {
      resolved.joints = new sensor_msgs.msg.JointState()
    }

    if (msg.isValid !== undefined) {
      resolved.isValid = msg.isValid;
    }
    else {
      resolved.isValid = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SolveIKRequest,
  Response: SolveIKResponse,
  md5sum() { return '135cec71206c75b4d25a0c184fe860d8'; },
  datatype() { return 'uuv_manipulators_msgs/SolveIK'; }
};
