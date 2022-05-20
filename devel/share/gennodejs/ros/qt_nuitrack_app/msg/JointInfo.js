// Auto-generated. Do not edit!

// (in-package qt_nuitrack_app.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class JointInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.type = null;
      this.confidence = null;
      this.real = null;
      this.projection = null;
      this.orientation = null;
    }
    else {
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('real')) {
        this.real = initObj.real
      }
      else {
        this.real = [];
      }
      if (initObj.hasOwnProperty('projection')) {
        this.projection = initObj.projection
      }
      else {
        this.projection = [];
      }
      if (initObj.hasOwnProperty('orientation')) {
        this.orientation = initObj.orientation
      }
      else {
        this.orientation = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointInfo
    // Serialize message field [type]
    bufferOffset = _serializer.uint8(obj.type, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    // Serialize message field [real]
    bufferOffset = _arraySerializer.float32(obj.real, buffer, bufferOffset, null);
    // Serialize message field [projection]
    bufferOffset = _arraySerializer.float32(obj.projection, buffer, bufferOffset, null);
    // Serialize message field [orientation]
    bufferOffset = _arraySerializer.float32(obj.orientation, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointInfo
    let len;
    let data = new JointInfo(null);
    // Deserialize message field [type]
    data.type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [real]
    data.real = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [projection]
    data.projection = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [orientation]
    data.orientation = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.real.length;
    length += 4 * object.projection.length;
    length += 4 * object.orientation.length;
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qt_nuitrack_app/JointInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '52ccea37fd6730979bc7bbe9557a5b06';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new JointInfo(null);
    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.real !== undefined) {
      resolved.real = msg.real;
    }
    else {
      resolved.real = []
    }

    if (msg.projection !== undefined) {
      resolved.projection = msg.projection;
    }
    else {
      resolved.projection = []
    }

    if (msg.orientation !== undefined) {
      resolved.orientation = msg.orientation;
    }
    else {
      resolved.orientation = []
    }

    return resolved;
    }
};

module.exports = JointInfo;
