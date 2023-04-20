// Auto-generated. Do not edit!

// (in-package adbot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SprMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isOn = null;
    }
    else {
      if (initObj.hasOwnProperty('isOn')) {
        this.isOn = initObj.isOn
      }
      else {
        this.isOn = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SprMsg
    // Serialize message field [isOn]
    bufferOffset = _serializer.bool(obj.isOn, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SprMsg
    let len;
    let data = new SprMsg(null);
    // Deserialize message field [isOn]
    data.isOn = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'adbot_msgs/SprMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7815df4e3625e249c3731540caef0458';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isOn
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SprMsg(null);
    if (msg.isOn !== undefined) {
      resolved.isOn = msg.isOn;
    }
    else {
      resolved.isOn = false
    }

    return resolved;
    }
};

module.exports = SprMsg;
