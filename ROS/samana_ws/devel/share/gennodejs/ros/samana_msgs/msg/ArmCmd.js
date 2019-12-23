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

class ArmCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.grabber_cmd = null;
      this.lifter_cmd = null;
    }
    else {
      if (initObj.hasOwnProperty('grabber_cmd')) {
        this.grabber_cmd = initObj.grabber_cmd
      }
      else {
        this.grabber_cmd = 0;
      }
      if (initObj.hasOwnProperty('lifter_cmd')) {
        this.lifter_cmd = initObj.lifter_cmd
      }
      else {
        this.lifter_cmd = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmCmd
    // Serialize message field [grabber_cmd]
    bufferOffset = _serializer.int8(obj.grabber_cmd, buffer, bufferOffset);
    // Serialize message field [lifter_cmd]
    bufferOffset = _serializer.int8(obj.lifter_cmd, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmCmd
    let len;
    let data = new ArmCmd(null);
    // Deserialize message field [grabber_cmd]
    data.grabber_cmd = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [lifter_cmd]
    data.lifter_cmd = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'samana_msgs/ArmCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4915ef1fb595c9707da4cb79c7caeeb8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 grabber_cmd
    int8 lifter_cmd
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmCmd(null);
    if (msg.grabber_cmd !== undefined) {
      resolved.grabber_cmd = msg.grabber_cmd;
    }
    else {
      resolved.grabber_cmd = 0
    }

    if (msg.lifter_cmd !== undefined) {
      resolved.lifter_cmd = msg.lifter_cmd;
    }
    else {
      resolved.lifter_cmd = 0
    }

    return resolved;
    }
};

module.exports = ArmCmd;
