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

class HandInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.right_projection = null;
      this.right_real = null;
      this.right_click = null;
      this.right_pressure = null;
      this.left_projection = null;
      this.left_real = null;
      this.left_click = null;
      this.left_pressure = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('right_projection')) {
        this.right_projection = initObj.right_projection
      }
      else {
        this.right_projection = [];
      }
      if (initObj.hasOwnProperty('right_real')) {
        this.right_real = initObj.right_real
      }
      else {
        this.right_real = [];
      }
      if (initObj.hasOwnProperty('right_click')) {
        this.right_click = initObj.right_click
      }
      else {
        this.right_click = false;
      }
      if (initObj.hasOwnProperty('right_pressure')) {
        this.right_pressure = initObj.right_pressure
      }
      else {
        this.right_pressure = 0;
      }
      if (initObj.hasOwnProperty('left_projection')) {
        this.left_projection = initObj.left_projection
      }
      else {
        this.left_projection = [];
      }
      if (initObj.hasOwnProperty('left_real')) {
        this.left_real = initObj.left_real
      }
      else {
        this.left_real = [];
      }
      if (initObj.hasOwnProperty('left_click')) {
        this.left_click = initObj.left_click
      }
      else {
        this.left_click = false;
      }
      if (initObj.hasOwnProperty('left_pressure')) {
        this.left_pressure = initObj.left_pressure
      }
      else {
        this.left_pressure = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HandInfo
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [right_projection]
    bufferOffset = _arraySerializer.float32(obj.right_projection, buffer, bufferOffset, null);
    // Serialize message field [right_real]
    bufferOffset = _arraySerializer.float32(obj.right_real, buffer, bufferOffset, null);
    // Serialize message field [right_click]
    bufferOffset = _serializer.bool(obj.right_click, buffer, bufferOffset);
    // Serialize message field [right_pressure]
    bufferOffset = _serializer.int32(obj.right_pressure, buffer, bufferOffset);
    // Serialize message field [left_projection]
    bufferOffset = _arraySerializer.float32(obj.left_projection, buffer, bufferOffset, null);
    // Serialize message field [left_real]
    bufferOffset = _arraySerializer.float32(obj.left_real, buffer, bufferOffset, null);
    // Serialize message field [left_click]
    bufferOffset = _serializer.bool(obj.left_click, buffer, bufferOffset);
    // Serialize message field [left_pressure]
    bufferOffset = _serializer.int32(obj.left_pressure, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HandInfo
    let len;
    let data = new HandInfo(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [right_projection]
    data.right_projection = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [right_real]
    data.right_real = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [right_click]
    data.right_click = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right_pressure]
    data.right_pressure = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [left_projection]
    data.left_projection = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [left_real]
    data.left_real = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [left_click]
    data.left_click = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [left_pressure]
    data.left_pressure = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.right_projection.length;
    length += 4 * object.right_real.length;
    length += 4 * object.left_projection.length;
    length += 4 * object.left_real.length;
    return length + 30;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qt_nuitrack_app/HandInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ebda9d77d8f24f2a507d651777ac1949';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new HandInfo(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.right_projection !== undefined) {
      resolved.right_projection = msg.right_projection;
    }
    else {
      resolved.right_projection = []
    }

    if (msg.right_real !== undefined) {
      resolved.right_real = msg.right_real;
    }
    else {
      resolved.right_real = []
    }

    if (msg.right_click !== undefined) {
      resolved.right_click = msg.right_click;
    }
    else {
      resolved.right_click = false
    }

    if (msg.right_pressure !== undefined) {
      resolved.right_pressure = msg.right_pressure;
    }
    else {
      resolved.right_pressure = 0
    }

    if (msg.left_projection !== undefined) {
      resolved.left_projection = msg.left_projection;
    }
    else {
      resolved.left_projection = []
    }

    if (msg.left_real !== undefined) {
      resolved.left_real = msg.left_real;
    }
    else {
      resolved.left_real = []
    }

    if (msg.left_click !== undefined) {
      resolved.left_click = msg.left_click;
    }
    else {
      resolved.left_click = false
    }

    if (msg.left_pressure !== undefined) {
      resolved.left_pressure = msg.left_pressure;
    }
    else {
      resolved.left_pressure = 0
    }

    return resolved;
    }
};

module.exports = HandInfo;
