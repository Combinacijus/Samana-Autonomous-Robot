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

class Sonic9 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Header = null;
      this.dist1 = null;
      this.dist2 = null;
      this.dist3 = null;
      this.dist4 = null;
      this.dist5 = null;
      this.dist6 = null;
      this.dist7 = null;
      this.dist8 = null;
      this.dist9 = null;
    }
    else {
      if (initObj.hasOwnProperty('Header')) {
        this.Header = initObj.Header
      }
      else {
        this.Header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('dist1')) {
        this.dist1 = initObj.dist1
      }
      else {
        this.dist1 = 0;
      }
      if (initObj.hasOwnProperty('dist2')) {
        this.dist2 = initObj.dist2
      }
      else {
        this.dist2 = 0;
      }
      if (initObj.hasOwnProperty('dist3')) {
        this.dist3 = initObj.dist3
      }
      else {
        this.dist3 = 0;
      }
      if (initObj.hasOwnProperty('dist4')) {
        this.dist4 = initObj.dist4
      }
      else {
        this.dist4 = 0;
      }
      if (initObj.hasOwnProperty('dist5')) {
        this.dist5 = initObj.dist5
      }
      else {
        this.dist5 = 0;
      }
      if (initObj.hasOwnProperty('dist6')) {
        this.dist6 = initObj.dist6
      }
      else {
        this.dist6 = 0;
      }
      if (initObj.hasOwnProperty('dist7')) {
        this.dist7 = initObj.dist7
      }
      else {
        this.dist7 = 0;
      }
      if (initObj.hasOwnProperty('dist8')) {
        this.dist8 = initObj.dist8
      }
      else {
        this.dist8 = 0;
      }
      if (initObj.hasOwnProperty('dist9')) {
        this.dist9 = initObj.dist9
      }
      else {
        this.dist9 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Sonic9
    // Serialize message field [Header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.Header, buffer, bufferOffset);
    // Serialize message field [dist1]
    bufferOffset = _serializer.int16(obj.dist1, buffer, bufferOffset);
    // Serialize message field [dist2]
    bufferOffset = _serializer.int16(obj.dist2, buffer, bufferOffset);
    // Serialize message field [dist3]
    bufferOffset = _serializer.int16(obj.dist3, buffer, bufferOffset);
    // Serialize message field [dist4]
    bufferOffset = _serializer.int16(obj.dist4, buffer, bufferOffset);
    // Serialize message field [dist5]
    bufferOffset = _serializer.int16(obj.dist5, buffer, bufferOffset);
    // Serialize message field [dist6]
    bufferOffset = _serializer.int16(obj.dist6, buffer, bufferOffset);
    // Serialize message field [dist7]
    bufferOffset = _serializer.int16(obj.dist7, buffer, bufferOffset);
    // Serialize message field [dist8]
    bufferOffset = _serializer.int16(obj.dist8, buffer, bufferOffset);
    // Serialize message field [dist9]
    bufferOffset = _serializer.int16(obj.dist9, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Sonic9
    let len;
    let data = new Sonic9(null);
    // Deserialize message field [Header]
    data.Header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [dist1]
    data.dist1 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [dist2]
    data.dist2 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [dist3]
    data.dist3 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [dist4]
    data.dist4 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [dist5]
    data.dist5 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [dist6]
    data.dist6 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [dist7]
    data.dist7 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [dist8]
    data.dist8 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [dist9]
    data.dist9 = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.Header);
    return length + 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'samana_msgs/Sonic9';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9475c4b12aca5b9b235ea1a9e9e22280';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header Header
    int16 dist1
    int16 dist2
    int16 dist3
    int16 dist4
    int16 dist5
    int16 dist6
    int16 dist7
    int16 dist8
    int16 dist9
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
    const resolved = new Sonic9(null);
    if (msg.Header !== undefined) {
      resolved.Header = std_msgs.msg.Header.Resolve(msg.Header)
    }
    else {
      resolved.Header = new std_msgs.msg.Header()
    }

    if (msg.dist1 !== undefined) {
      resolved.dist1 = msg.dist1;
    }
    else {
      resolved.dist1 = 0
    }

    if (msg.dist2 !== undefined) {
      resolved.dist2 = msg.dist2;
    }
    else {
      resolved.dist2 = 0
    }

    if (msg.dist3 !== undefined) {
      resolved.dist3 = msg.dist3;
    }
    else {
      resolved.dist3 = 0
    }

    if (msg.dist4 !== undefined) {
      resolved.dist4 = msg.dist4;
    }
    else {
      resolved.dist4 = 0
    }

    if (msg.dist5 !== undefined) {
      resolved.dist5 = msg.dist5;
    }
    else {
      resolved.dist5 = 0
    }

    if (msg.dist6 !== undefined) {
      resolved.dist6 = msg.dist6;
    }
    else {
      resolved.dist6 = 0
    }

    if (msg.dist7 !== undefined) {
      resolved.dist7 = msg.dist7;
    }
    else {
      resolved.dist7 = 0
    }

    if (msg.dist8 !== undefined) {
      resolved.dist8 = msg.dist8;
    }
    else {
      resolved.dist8 = 0
    }

    if (msg.dist9 !== undefined) {
      resolved.dist9 = msg.dist9;
    }
    else {
      resolved.dist9 = 0
    }

    return resolved;
    }
};

module.exports = Sonic9;
