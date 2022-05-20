// Auto-generated. Do not edit!

// (in-package qt_nuitrack_app.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let HandInfo = require('./HandInfo.js');

//-----------------------------------------------------------

class Hands {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.hands = null;
    }
    else {
      if (initObj.hasOwnProperty('hands')) {
        this.hands = initObj.hands
      }
      else {
        this.hands = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Hands
    // Serialize message field [hands]
    // Serialize the length for message field [hands]
    bufferOffset = _serializer.uint32(obj.hands.length, buffer, bufferOffset);
    obj.hands.forEach((val) => {
      bufferOffset = HandInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Hands
    let len;
    let data = new Hands(null);
    // Deserialize message field [hands]
    // Deserialize array length for message field [hands]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.hands = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.hands[i] = HandInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.hands.forEach((val) => {
      length += HandInfo.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qt_nuitrack_app/Hands';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '633d76b336567f0906335e6cf0195299';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #std_msgs/Header header
    HandInfo[] hands
    
    ================================================================================
    MSG: qt_nuitrack_app/HandInfo
    #std_msgs/Header header
    int32 id
    #The normalized projective (x, y) coordinate of the right hand
    float32[] right_projection
    #The (x,y,z) coordinate of the right hand in the world system.
    float32[] right_real
    bool right_click
    int32 right_pressure
    #The normalized projective (x, y) coordinate of the left hand
    float32[] left_projection
    #The (x,y,z) coordinate of the left hand in the world system.
    float32[] left_real
    bool left_click
    int32 left_pressure
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Hands(null);
    if (msg.hands !== undefined) {
      resolved.hands = new Array(msg.hands.length);
      for (let i = 0; i < resolved.hands.length; ++i) {
        resolved.hands[i] = HandInfo.Resolve(msg.hands[i]);
      }
    }
    else {
      resolved.hands = []
    }

    return resolved;
    }
};

module.exports = Hands;
