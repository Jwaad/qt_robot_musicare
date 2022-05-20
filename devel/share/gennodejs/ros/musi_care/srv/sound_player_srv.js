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

class sound_player_srvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.operation = null;
      this.operation_data_1 = null;
      this.operation_data_2 = null;
    }
    else {
      if (initObj.hasOwnProperty('operation')) {
        this.operation = initObj.operation
      }
      else {
        this.operation = '';
      }
      if (initObj.hasOwnProperty('operation_data_1')) {
        this.operation_data_1 = initObj.operation_data_1
      }
      else {
        this.operation_data_1 = '';
      }
      if (initObj.hasOwnProperty('operation_data_2')) {
        this.operation_data_2 = initObj.operation_data_2
      }
      else {
        this.operation_data_2 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sound_player_srvRequest
    // Serialize message field [operation]
    bufferOffset = _serializer.string(obj.operation, buffer, bufferOffset);
    // Serialize message field [operation_data_1]
    bufferOffset = _serializer.string(obj.operation_data_1, buffer, bufferOffset);
    // Serialize message field [operation_data_2]
    bufferOffset = _serializer.float32(obj.operation_data_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sound_player_srvRequest
    let len;
    let data = new sound_player_srvRequest(null);
    // Deserialize message field [operation]
    data.operation = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [operation_data_1]
    data.operation_data_1 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [operation_data_2]
    data.operation_data_2 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.operation);
    length += _getByteLength(object.operation_data_1);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'musi_care/sound_player_srvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0c2f2cc2cd7d8db676e682d44ac19b31';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string operation
    string operation_data_1
    float32 operation_data_2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sound_player_srvRequest(null);
    if (msg.operation !== undefined) {
      resolved.operation = msg.operation;
    }
    else {
      resolved.operation = ''
    }

    if (msg.operation_data_1 !== undefined) {
      resolved.operation_data_1 = msg.operation_data_1;
    }
    else {
      resolved.operation_data_1 = ''
    }

    if (msg.operation_data_2 !== undefined) {
      resolved.operation_data_2 = msg.operation_data_2;
    }
    else {
      resolved.operation_data_2 = 0.0
    }

    return resolved;
    }
};

class sound_player_srvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.track_title = null;
      this.track_elapsed_time = null;
      this.track_total_time = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('track_title')) {
        this.track_title = initObj.track_title
      }
      else {
        this.track_title = '';
      }
      if (initObj.hasOwnProperty('track_elapsed_time')) {
        this.track_elapsed_time = initObj.track_elapsed_time
      }
      else {
        this.track_elapsed_time = 0.0;
      }
      if (initObj.hasOwnProperty('track_total_time')) {
        this.track_total_time = initObj.track_total_time
      }
      else {
        this.track_total_time = 0.0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sound_player_srvResponse
    // Serialize message field [track_title]
    bufferOffset = _serializer.string(obj.track_title, buffer, bufferOffset);
    // Serialize message field [track_elapsed_time]
    bufferOffset = _serializer.float32(obj.track_elapsed_time, buffer, bufferOffset);
    // Serialize message field [track_total_time]
    bufferOffset = _serializer.float32(obj.track_total_time, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sound_player_srvResponse
    let len;
    let data = new sound_player_srvResponse(null);
    // Deserialize message field [track_title]
    data.track_title = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [track_elapsed_time]
    data.track_elapsed_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [track_total_time]
    data.track_total_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.track_title);
    return length + 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'musi_care/sound_player_srvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a58a3e4eb14adf57544e531765e60a08';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string track_title
    float32 track_elapsed_time
    float32 track_total_time
    bool status
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sound_player_srvResponse(null);
    if (msg.track_title !== undefined) {
      resolved.track_title = msg.track_title;
    }
    else {
      resolved.track_title = ''
    }

    if (msg.track_elapsed_time !== undefined) {
      resolved.track_elapsed_time = msg.track_elapsed_time;
    }
    else {
      resolved.track_elapsed_time = 0.0
    }

    if (msg.track_total_time !== undefined) {
      resolved.track_total_time = msg.track_total_time;
    }
    else {
      resolved.track_total_time = 0.0
    }

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
  Request: sound_player_srvRequest,
  Response: sound_player_srvResponse,
  md5sum() { return '2a3bef821864039a35e20805c207d7a8'; },
  datatype() { return 'musi_care/sound_player_srv'; }
};
