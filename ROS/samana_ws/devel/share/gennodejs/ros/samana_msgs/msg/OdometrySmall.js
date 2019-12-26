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
      this.delta_ticks1 = null;
      this.delta_ticks2 = null;
      this.rps1 = null;
      this.rps2 = null;
      this.dt = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('delta_ticks1')) {
        this.delta_ticks1 = initObj.delta_ticks1
      }
      else {
        this.delta_ticks1 = 0;
      }
      if (initObj.hasOwnProperty('delta_ticks2')) {
        this.delta_ticks2 = initObj.delta_ticks2
      }
      else {
        this.delta_ticks2 = 0;
      }
      if (initObj.hasOwnProperty('rps1')) {
        this.rps1 = initObj.rps1
      }
      else {
        this.rps1 = 0.0;
      }
      if (initObj.hasOwnProperty('rps2')) {
        this.rps2 = initObj.rps2
      }
      else {
        this.rps2 = 0.0;
      }
      if (initObj.hasOwnProperty('dt')) {
        this.dt = initObj.dt
      }
      else {
        this.dt = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OdometrySmall
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [delta_ticks1]
    bufferOffset = _serializer.int16(obj.delta_ticks1, buffer, bufferOffset);
    // Serialize message field [delta_ticks2]
    bufferOffset = _serializer.int16(obj.delta_ticks2, buffer, bufferOffset);
    // Serialize message field [rps1]
    bufferOffset = _serializer.float32(obj.rps1, buffer, bufferOffset);
    // Serialize message field [rps2]
    bufferOffset = _serializer.float32(obj.rps2, buffer, bufferOffset);
    // Serialize message field [dt]
    bufferOffset = _serializer.int16(obj.dt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OdometrySmall
    let len;
    let data = new OdometrySmall(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [delta_ticks1]
    data.delta_ticks1 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [delta_ticks2]
    data.delta_ticks2 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [rps1]
    data.rps1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rps2]
    data.rps2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dt]
    data.dt = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 14;
  }

  static datatype() {
    // Returns string type for a message object
    return 'samana_msgs/OdometrySmall';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '98004b29be7f03ed0afbf8488b6e7875';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    int16 delta_ticks1
    int16 delta_ticks2
    float32 rps1
    float32 rps2
    int16 dt
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

    if (msg.delta_ticks1 !== undefined) {
      resolved.delta_ticks1 = msg.delta_ticks1;
    }
    else {
      resolved.delta_ticks1 = 0
    }

    if (msg.delta_ticks2 !== undefined) {
      resolved.delta_ticks2 = msg.delta_ticks2;
    }
    else {
      resolved.delta_ticks2 = 0
    }

    if (msg.rps1 !== undefined) {
      resolved.rps1 = msg.rps1;
    }
    else {
      resolved.rps1 = 0.0
    }

    if (msg.rps2 !== undefined) {
      resolved.rps2 = msg.rps2;
    }
    else {
      resolved.rps2 = 0.0
    }

    if (msg.dt !== undefined) {
      resolved.dt = msg.dt;
    }
    else {
      resolved.dt = 0
    }

    return resolved;
    }
};

module.exports = OdometrySmall;
