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

class ImuCalib {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sys = null;
      this.gyr = null;
      this.acc = null;
      this.mag = null;
      this.temp = null;
    }
    else {
      if (initObj.hasOwnProperty('sys')) {
        this.sys = initObj.sys
      }
      else {
        this.sys = 0;
      }
      if (initObj.hasOwnProperty('gyr')) {
        this.gyr = initObj.gyr
      }
      else {
        this.gyr = 0;
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = 0;
      }
      if (initObj.hasOwnProperty('mag')) {
        this.mag = initObj.mag
      }
      else {
        this.mag = 0;
      }
      if (initObj.hasOwnProperty('temp')) {
        this.temp = initObj.temp
      }
      else {
        this.temp = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ImuCalib
    // Serialize message field [sys]
    bufferOffset = _serializer.uint8(obj.sys, buffer, bufferOffset);
    // Serialize message field [gyr]
    bufferOffset = _serializer.uint8(obj.gyr, buffer, bufferOffset);
    // Serialize message field [acc]
    bufferOffset = _serializer.uint8(obj.acc, buffer, bufferOffset);
    // Serialize message field [mag]
    bufferOffset = _serializer.uint8(obj.mag, buffer, bufferOffset);
    // Serialize message field [temp]
    bufferOffset = _serializer.int8(obj.temp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ImuCalib
    let len;
    let data = new ImuCalib(null);
    // Deserialize message field [sys]
    data.sys = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gyr]
    data.gyr = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [acc]
    data.acc = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [mag]
    data.mag = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [temp]
    data.temp = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'samana_msgs/ImuCalib';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7764c0234b4e443d9ef754fa0119997d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 sys
    uint8 gyr
    uint8 acc
    uint8 mag
    int8 temp
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ImuCalib(null);
    if (msg.sys !== undefined) {
      resolved.sys = msg.sys;
    }
    else {
      resolved.sys = 0
    }

    if (msg.gyr !== undefined) {
      resolved.gyr = msg.gyr;
    }
    else {
      resolved.gyr = 0
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = 0
    }

    if (msg.mag !== undefined) {
      resolved.mag = msg.mag;
    }
    else {
      resolved.mag = 0
    }

    if (msg.temp !== undefined) {
      resolved.temp = msg.temp;
    }
    else {
      resolved.temp = 0
    }

    return resolved;
    }
};

module.exports = ImuCalib;
