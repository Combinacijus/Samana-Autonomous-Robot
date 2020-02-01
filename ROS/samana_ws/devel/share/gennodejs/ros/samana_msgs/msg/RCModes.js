// Auto-generated. Do not edit!

// (in-package samana_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class RCModes {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.allow_rc = null;
      this.armed = null;
      this.auton_mode = null;
      this.arm_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('allow_rc')) {
        this.allow_rc = initObj.allow_rc
      }
      else {
        this.allow_rc = false;
      }
      if (initObj.hasOwnProperty('armed')) {
        this.armed = initObj.armed
      }
      else {
        this.armed = false;
      }
      if (initObj.hasOwnProperty('auton_mode')) {
        this.auton_mode = initObj.auton_mode
      }
      else {
        this.auton_mode = false;
      }
      if (initObj.hasOwnProperty('arm_mode')) {
        this.arm_mode = initObj.arm_mode
      }
      else {
        this.arm_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RCModes
    // Serialize message field [allow_rc]
    bufferOffset = _serializer.bool(obj.allow_rc, buffer, bufferOffset);
    // Serialize message field [armed]
    bufferOffset = _serializer.bool(obj.armed, buffer, bufferOffset);
    // Serialize message field [auton_mode]
    bufferOffset = _serializer.bool(obj.auton_mode, buffer, bufferOffset);
    // Serialize message field [arm_mode]
    bufferOffset = _serializer.uint8(obj.arm_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RCModes
    let len;
    let data = new RCModes(null);
    // Deserialize message field [allow_rc]
    data.allow_rc = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [armed]
    data.armed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [auton_mode]
    data.auton_mode = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [arm_mode]
    data.arm_mode = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'samana_msgs/RCModes';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f93b20a9c6f7b0344addef9c5a6daf0f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool allow_rc
    bool armed
    bool auton_mode
    uint8 arm_mode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RCModes(null);
    if (msg.allow_rc !== undefined) {
      resolved.allow_rc = msg.allow_rc;
    }
    else {
      resolved.allow_rc = false
    }

    if (msg.armed !== undefined) {
      resolved.armed = msg.armed;
    }
    else {
      resolved.armed = false
    }

    if (msg.auton_mode !== undefined) {
      resolved.auton_mode = msg.auton_mode;
    }
    else {
      resolved.auton_mode = false
    }

    if (msg.arm_mode !== undefined) {
      resolved.arm_mode = msg.arm_mode;
    }
    else {
      resolved.arm_mode = 0
    }

    return resolved;
    }
};

module.exports = RCModes;
