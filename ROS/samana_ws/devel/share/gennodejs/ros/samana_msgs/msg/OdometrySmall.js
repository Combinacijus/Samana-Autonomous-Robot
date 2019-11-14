// Auto-generated. Do not edit!

// (in-package samana_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class OdometrySmall {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ticks1 = null;
      this.ticks2 = null;
      this.speed1 = null;
      this.speed2 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('ticks1')) {
        this.ticks1 = initObj.ticks1
      }
      else {
        this.ticks1 = 0;
      }
      if (initObj.hasOwnProperty('ticks2')) {
        this.ticks2 = initObj.ticks2
      }
      else {
        this.ticks2 = 0;
      }
      if (initObj.hasOwnProperty('speed1')) {
        this.speed1 = initObj.speed1
      }
      else {
        this.speed1 = 0.0;
      }
      if (initObj.hasOwnProperty('speed2')) {
        this.speed2 = initObj.speed2
      }
      else {
        this.speed2 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OdometrySmall
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ticks1]
    bufferOffset = _serializer.uint16(obj.ticks1, buffer, bufferOffset);
    // Serialize message field [ticks2]
    bufferOffset = _serializer.uint16(obj.ticks2, buffer, bufferOffset);
    // Serialize message field [speed1]
    bufferOffset = _serializer.float32(obj.speed1, buffer, bufferOffset);
    // Serialize message field [speed2]
    bufferOffset = _serializer.float32(obj.speed2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OdometrySmall
    let len;
    let data = new OdometrySmall(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ticks1]
    data.ticks1 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [ticks2]
    data.ticks2 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [speed1]
    data.speed1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed2]
    data.speed2 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'samana_msgs/OdometrySmall';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '66d5c83f5380a539385185906d186ae9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    uint16 ticks1
    uint16 ticks2
    float32 speed1
    float32 speed2
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
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OdometrySmall(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.ticks1 !== undefined) {
      resolved.ticks1 = msg.ticks1;
    }
    else {
      resolved.ticks1 = 0
    }

    if (msg.ticks2 !== undefined) {
      resolved.ticks2 = msg.ticks2;
    }
    else {
      resolved.ticks2 = 0
    }

    if (msg.speed1 !== undefined) {
      resolved.speed1 = msg.speed1;
    }
    else {
      resolved.speed1 = 0.0
    }

    if (msg.speed2 !== undefined) {
      resolved.speed2 = msg.speed2;
    }
    else {
      resolved.speed2 = 0.0
    }

    return resolved;
    }
};

module.exports = OdometrySmall;
