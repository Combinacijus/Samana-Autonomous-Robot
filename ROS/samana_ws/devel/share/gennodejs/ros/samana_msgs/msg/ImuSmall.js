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

class ImuSmall {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.quaternion_x = null;
      this.quaternion_y = null;
      this.quaternion_z = null;
      this.quaternion_w = null;
      this.linear_acceleration_x = null;
      this.linear_acceleration_y = null;
      this.linear_acceleration_z = null;
      this.angular_velocity_x = null;
      this.angular_velocity_y = null;
      this.angular_velocity_z = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('quaternion_x')) {
        this.quaternion_x = initObj.quaternion_x
      }
      else {
        this.quaternion_x = 0.0;
      }
      if (initObj.hasOwnProperty('quaternion_y')) {
        this.quaternion_y = initObj.quaternion_y
      }
      else {
        this.quaternion_y = 0.0;
      }
      if (initObj.hasOwnProperty('quaternion_z')) {
        this.quaternion_z = initObj.quaternion_z
      }
      else {
        this.quaternion_z = 0.0;
      }
      if (initObj.hasOwnProperty('quaternion_w')) {
        this.quaternion_w = initObj.quaternion_w
      }
      else {
        this.quaternion_w = 0.0;
      }
      if (initObj.hasOwnProperty('linear_acceleration_x')) {
        this.linear_acceleration_x = initObj.linear_acceleration_x
      }
      else {
        this.linear_acceleration_x = 0.0;
      }
      if (initObj.hasOwnProperty('linear_acceleration_y')) {
        this.linear_acceleration_y = initObj.linear_acceleration_y
      }
      else {
        this.linear_acceleration_y = 0.0;
      }
      if (initObj.hasOwnProperty('linear_acceleration_z')) {
        this.linear_acceleration_z = initObj.linear_acceleration_z
      }
      else {
        this.linear_acceleration_z = 0.0;
      }
      if (initObj.hasOwnProperty('angular_velocity_x')) {
        this.angular_velocity_x = initObj.angular_velocity_x
      }
      else {
        this.angular_velocity_x = 0.0;
      }
      if (initObj.hasOwnProperty('angular_velocity_y')) {
        this.angular_velocity_y = initObj.angular_velocity_y
      }
      else {
        this.angular_velocity_y = 0.0;
      }
      if (initObj.hasOwnProperty('angular_velocity_z')) {
        this.angular_velocity_z = initObj.angular_velocity_z
      }
      else {
        this.angular_velocity_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ImuSmall
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [quaternion_x]
    bufferOffset = _serializer.float32(obj.quaternion_x, buffer, bufferOffset);
    // Serialize message field [quaternion_y]
    bufferOffset = _serializer.float32(obj.quaternion_y, buffer, bufferOffset);
    // Serialize message field [quaternion_z]
    bufferOffset = _serializer.float32(obj.quaternion_z, buffer, bufferOffset);
    // Serialize message field [quaternion_w]
    bufferOffset = _serializer.float32(obj.quaternion_w, buffer, bufferOffset);
    // Serialize message field [linear_acceleration_x]
    bufferOffset = _serializer.float32(obj.linear_acceleration_x, buffer, bufferOffset);
    // Serialize message field [linear_acceleration_y]
    bufferOffset = _serializer.float32(obj.linear_acceleration_y, buffer, bufferOffset);
    // Serialize message field [linear_acceleration_z]
    bufferOffset = _serializer.float32(obj.linear_acceleration_z, buffer, bufferOffset);
    // Serialize message field [angular_velocity_x]
    bufferOffset = _serializer.float32(obj.angular_velocity_x, buffer, bufferOffset);
    // Serialize message field [angular_velocity_y]
    bufferOffset = _serializer.float32(obj.angular_velocity_y, buffer, bufferOffset);
    // Serialize message field [angular_velocity_z]
    bufferOffset = _serializer.float32(obj.angular_velocity_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ImuSmall
    let len;
    let data = new ImuSmall(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [quaternion_x]
    data.quaternion_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [quaternion_y]
    data.quaternion_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [quaternion_z]
    data.quaternion_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [quaternion_w]
    data.quaternion_w = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [linear_acceleration_x]
    data.linear_acceleration_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [linear_acceleration_y]
    data.linear_acceleration_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [linear_acceleration_z]
    data.linear_acceleration_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular_velocity_x]
    data.angular_velocity_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular_velocity_y]
    data.angular_velocity_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular_velocity_z]
    data.angular_velocity_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'samana_msgs/ImuSmall';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c052128fb9568718800fa0ba7071e271';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    float32 quaternion_x
    float32 quaternion_y
    float32 quaternion_z
    float32 quaternion_w
    float32 linear_acceleration_x
    float32 linear_acceleration_y
    float32 linear_acceleration_z
    float32 angular_velocity_x
    float32 angular_velocity_y
    float32 angular_velocity_z
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
    const resolved = new ImuSmall(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.quaternion_x !== undefined) {
      resolved.quaternion_x = msg.quaternion_x;
    }
    else {
      resolved.quaternion_x = 0.0
    }

    if (msg.quaternion_y !== undefined) {
      resolved.quaternion_y = msg.quaternion_y;
    }
    else {
      resolved.quaternion_y = 0.0
    }

    if (msg.quaternion_z !== undefined) {
      resolved.quaternion_z = msg.quaternion_z;
    }
    else {
      resolved.quaternion_z = 0.0
    }

    if (msg.quaternion_w !== undefined) {
      resolved.quaternion_w = msg.quaternion_w;
    }
    else {
      resolved.quaternion_w = 0.0
    }

    if (msg.linear_acceleration_x !== undefined) {
      resolved.linear_acceleration_x = msg.linear_acceleration_x;
    }
    else {
      resolved.linear_acceleration_x = 0.0
    }

    if (msg.linear_acceleration_y !== undefined) {
      resolved.linear_acceleration_y = msg.linear_acceleration_y;
    }
    else {
      resolved.linear_acceleration_y = 0.0
    }

    if (msg.linear_acceleration_z !== undefined) {
      resolved.linear_acceleration_z = msg.linear_acceleration_z;
    }
    else {
      resolved.linear_acceleration_z = 0.0
    }

    if (msg.angular_velocity_x !== undefined) {
      resolved.angular_velocity_x = msg.angular_velocity_x;
    }
    else {
      resolved.angular_velocity_x = 0.0
    }

    if (msg.angular_velocity_y !== undefined) {
      resolved.angular_velocity_y = msg.angular_velocity_y;
    }
    else {
      resolved.angular_velocity_y = 0.0
    }

    if (msg.angular_velocity_z !== undefined) {
      resolved.angular_velocity_z = msg.angular_velocity_z;
    }
    else {
      resolved.angular_velocity_z = 0.0
    }

    return resolved;
    }
};

module.exports = ImuSmall;
