// Auto-generated. Do not edit!

// (in-package fabot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ArmMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.hand = null;
      this.arm = null;
      this.hand_duty = null;
      this.arm_duty = null;
    }
    else {
      if (initObj.hasOwnProperty('hand')) {
        this.hand = initObj.hand
      }
      else {
        this.hand = 0;
      }
      if (initObj.hasOwnProperty('arm')) {
        this.arm = initObj.arm
      }
      else {
        this.arm = 0;
      }
      if (initObj.hasOwnProperty('hand_duty')) {
        this.hand_duty = initObj.hand_duty
      }
      else {
        this.hand_duty = 0;
      }
      if (initObj.hasOwnProperty('arm_duty')) {
        this.arm_duty = initObj.arm_duty
      }
      else {
        this.arm_duty = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmMsg
    // Serialize message field [hand]
    bufferOffset = _serializer.int16(obj.hand, buffer, bufferOffset);
    // Serialize message field [arm]
    bufferOffset = _serializer.int16(obj.arm, buffer, bufferOffset);
    // Serialize message field [hand_duty]
    bufferOffset = _serializer.int16(obj.hand_duty, buffer, bufferOffset);
    // Serialize message field [arm_duty]
    bufferOffset = _serializer.int16(obj.arm_duty, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmMsg
    let len;
    let data = new ArmMsg(null);
    // Deserialize message field [hand]
    data.hand = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [arm]
    data.arm = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [hand_duty]
    data.hand_duty = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [arm_duty]
    data.arm_duty = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fabot_msgs/ArmMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7e5814a5ce25cfec12c99674e013cee3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 hand
    int16 arm
    int16 hand_duty
    int16 arm_duty
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmMsg(null);
    if (msg.hand !== undefined) {
      resolved.hand = msg.hand;
    }
    else {
      resolved.hand = 0
    }

    if (msg.arm !== undefined) {
      resolved.arm = msg.arm;
    }
    else {
      resolved.arm = 0
    }

    if (msg.hand_duty !== undefined) {
      resolved.hand_duty = msg.hand_duty;
    }
    else {
      resolved.hand_duty = 0
    }

    if (msg.arm_duty !== undefined) {
      resolved.arm_duty = msg.arm_duty;
    }
    else {
      resolved.arm_duty = 0
    }

    return resolved;
    }
};

module.exports = ArmMsg;
