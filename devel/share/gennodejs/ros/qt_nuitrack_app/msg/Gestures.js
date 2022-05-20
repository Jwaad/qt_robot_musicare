// Auto-generated. Do not edit!

// (in-package qt_nuitrack_app.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let GestureInfo = require('./GestureInfo.js');

//-----------------------------------------------------------

class Gestures {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gestures = null;
    }
    else {
      if (initObj.hasOwnProperty('gestures')) {
        this.gestures = initObj.gestures
      }
      else {
        this.gestures = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Gestures
    // Serialize message field [gestures]
    // Serialize the length for message field [gestures]
    bufferOffset = _serializer.uint32(obj.gestures.length, buffer, bufferOffset);
    obj.gestures.forEach((val) => {
      bufferOffset = GestureInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Gestures
    let len;
    let data = new Gestures(null);
    // Deserialize message field [gestures]
    // Deserialize array length for message field [gestures]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.gestures = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.gestures[i] = GestureInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.gestures.forEach((val) => {
      length += GestureInfo.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qt_nuitrack_app/Gestures';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cb33e599df6d94bfd33170149d8d8cba';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #std_msgs/Header header
    GestureInfo[] gestures
    
    ================================================================================
    MSG: qt_nuitrack_app/GestureInfo
    #std_msgs/Header header
    int32 id
    string name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Gestures(null);
    if (msg.gestures !== undefined) {
      resolved.gestures = new Array(msg.gestures.length);
      for (let i = 0; i < resolved.gestures.length; ++i) {
        resolved.gestures[i] = GestureInfo.Resolve(msg.gestures[i]);
      }
    }
    else {
      resolved.gestures = []
    }

    return resolved;
    }
};

module.exports = Gestures;
