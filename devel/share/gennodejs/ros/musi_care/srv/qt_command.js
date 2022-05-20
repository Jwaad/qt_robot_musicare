// Auto-generated. Do not edit!

// (in-package musi_care.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class qt_commandRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.action_type = null;
      this.action_content = null;
      this.action_blocking = null;
    }
    else {
      if (initObj.hasOwnProperty('action_type')) {
        this.action_type = initObj.action_type
      }
      else {
        this.action_type = '';
      }
      if (initObj.hasOwnProperty('action_content')) {
        this.action_content = initObj.action_content
      }
      else {
        this.action_content = '';
      }
      if (initObj.hasOwnProperty('action_blocking')) {
        this.action_blocking = initObj.action_blocking
      }
      else {
        this.action_blocking = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type qt_commandRequest
    // Serialize message field [action_type]
    bufferOffset = _serializer.string(obj.action_type, buffer, bufferOffset);
    // Serialize message field [action_content]
    bufferOffset = _serializer.string(obj.action_content, buffer, bufferOffset);
    // Serialize message field [action_blocking]
    bufferOffset = _serializer.bool(obj.action_blocking, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type qt_commandRequest
    let len;
    let data = new qt_commandRequest(null);
    // Deserialize message field [action_type]
    data.action_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [action_content]
    data.action_content = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [action_blocking]
    data.action_blocking = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.action_type);
    length += _getByteLength(object.action_content);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'musi_care/qt_commandRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8c1433307c282c48513732c9c0efec5a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string action_type
    string action_content
    bool action_blocking
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new qt_commandRequest(null);
    if (msg.action_type !== undefined) {
      resolved.action_type = msg.action_type;
    }
    else {
      resolved.action_type = ''
    }

    if (msg.action_content !== undefined) {
      resolved.action_content = msg.action_content;
    }
    else {
      resolved.action_content = ''
    }

    if (msg.action_blocking !== undefined) {
      resolved.action_blocking = msg.action_blocking;
    }
    else {
      resolved.action_blocking = false
    }

    return resolved;
    }
};

class qt_commandResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type qt_commandResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type qt_commandResponse
    let len;
    let data = new qt_commandResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'musi_care/qt_commandResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3a1255d4d998bd4d6585c64639b5ee9a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool status
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new qt_commandResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = false
    }

    return resolved;
    }
};

module.exports = {
  Request: qt_commandRequest,
  Response: qt_commandResponse,
  md5sum() { return '63c073458379b344b40c9cac0c2eacff'; },
  datatype() { return 'musi_care/qt_command'; }
};
