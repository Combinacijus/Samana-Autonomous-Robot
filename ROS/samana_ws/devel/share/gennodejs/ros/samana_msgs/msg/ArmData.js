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

class ArmData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.current_grabber = null;
      this.current_lifter = null;
      this.limit_switches = null;
    }
    else {
      if (initObj.hasOwnProperty('current_grabber')) {
        this.current_grabber = initObj.current_grabber
      }
      else {
        this.current_grabber = 0;
      }
      if (initObj.hasOwnProperty('current_lifter')) {
        this.current_lifter = initObj.current_lifter
      }
      else {
        this.current_lifter = 0;
      }
      if (initObj.hasOwnProperty('limit_switches')) {
        this.limit_switches = initObj.limit_switches
      }
      else {
        this.limit_switches = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmData
    // Serialize message field [current_grabber]
    bufferOffset = _serializer.int16(obj.current_grabber, buffer, bufferOffset);
    // Serialize message field [current_lifter]
    bufferOffset = _serializer.int16(obj.current_lifter, buffer, bufferOffset);
    // Serialize message field [limit_switches]
    bufferOffset = _serializer.uint8(obj.limit_switches, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmData
    let len;
    let data = new ArmData(null);
    // Deserialize message field [current_grabber]
    data.current_grabber = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [current_lifter]
    data.current_lifter = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [limit_switches]
    data.limit_switches = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'samana_msgs/ArmData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3812456ac8efcba0c0c24a88991ba799';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 current_grabber
    int16 current_lifter
    uint8 limit_switches
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmData(null);
    if (msg.current_grabber !== undefined) {
      resolved.current_grabber = msg.current_grabber;
    }
    else {
      resolved.current_grabber = 0
    }

    if (msg.current_lifter !== undefined) {
      resolved.current_lifter = msg.current_lifter;
    }
    else {
      resolved.current_lifter = 0
    }

    if (msg.limit_switches !== undefined) {
      resolved.limit_switches = msg.limit_switches;
    }
    else {
      resolved.limit_switches = 0
    }

    return resolved;
    }
};

module.exports = ArmData;
