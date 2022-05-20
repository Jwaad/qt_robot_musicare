// Auto-generated. Do not edit!

// (in-package qt_nuitrack_app.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SkeletonInfo = require('./SkeletonInfo.js');

//-----------------------------------------------------------

class Skeletons {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.skeletons = null;
    }
    else {
      if (initObj.hasOwnProperty('skeletons')) {
        this.skeletons = initObj.skeletons
      }
      else {
        this.skeletons = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Skeletons
    // Serialize message field [skeletons]
    // Serialize the length for message field [skeletons]
    bufferOffset = _serializer.uint32(obj.skeletons.length, buffer, bufferOffset);
    obj.skeletons.forEach((val) => {
      bufferOffset = SkeletonInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Skeletons
    let len;
    let data = new Skeletons(null);
    // Deserialize message field [skeletons]
    // Deserialize array length for message field [skeletons]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.skeletons = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.skeletons[i] = SkeletonInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.skeletons.forEach((val) => {
      length += SkeletonInfo.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qt_nuitrack_app/Skeletons';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c9e6a0abe2b44381a0de593ee0c2c308';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #std_msgs/Header header
    SkeletonInfo[] skeletons
    
    ================================================================================
    MSG: qt_nuitrack_app/SkeletonInfo
    #std_msgs/Header header
    int32 id
    JointInfo[] joints 
    
    ================================================================================
    MSG: qt_nuitrack_app/JointInfo
    #std_msgs/Header header
    uint8 type
    float32 confidence
    float32[] real
    float32[] projection
    float32[] orientation
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Skeletons(null);
    if (msg.skeletons !== undefined) {
      resolved.skeletons = new Array(msg.skeletons.length);
      for (let i = 0; i < resolved.skeletons.length; ++i) {
        resolved.skeletons[i] = SkeletonInfo.Resolve(msg.skeletons[i]);
      }
    }
    else {
      resolved.skeletons = []
    }

    return resolved;
    }
};

module.exports = Skeletons;
