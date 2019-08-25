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

class Teleop {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.speed = null;
      this.steer = null;
    }
    else {
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0;
      }
      if (initObj.hasOwnProperty('steer')) {
        this.steer = initObj.steer
      }
      else {
        this.steer = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Teleop
    // Serialize message field [speed]
    bufferOffset = _serializer.int16(obj.speed, buffer, bufferOffset);
    // Serialize message field [steer]
    bufferOffset = _serializer.int16(obj.steer, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Teleop
    let len;
    let data = new Teleop(null);
    // Deserialize message field [speed]
    data.speed = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [steer]
    data.steer = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'samana_msgs/Teleop';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8b9ad615ef7ce8cbee2931ec476027b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 speed
    int16 steer
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Teleop(null);
    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0
    }

    if (msg.steer !== undefined) {
      resolved.steer = msg.steer;
    }
    else {
      resolved.steer = 0
    }

    return resolved;
    }
};

module.exports = Teleop;
