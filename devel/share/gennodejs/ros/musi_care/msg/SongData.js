// Auto-generated. Do not edit!

// (in-package musi_care.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SongData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.track_title = null;
      this.track_elapsed_time = null;
      this.track_total_time = null;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SongData
    // Serialize message field [track_title]
    bufferOffset = _serializer.string(obj.track_title, buffer, bufferOffset);
    // Serialize message field [track_elapsed_time]
    bufferOffset = _serializer.float32(obj.track_elapsed_time, buffer, bufferOffset);
    // Serialize message field [track_total_time]
    bufferOffset = _serializer.float32(obj.track_total_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SongData
    let len;
    let data = new SongData(null);
    // Deserialize message field [track_title]
    data.track_title = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [track_elapsed_time]
    data.track_elapsed_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [track_total_time]
    data.track_total_time = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.track_title);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'musi_care/SongData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '82324e20ca3e42f8d6a8f9e9503524ba';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string track_title
    float32 track_elapsed_time
    float32 track_total_time
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SongData(null);
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

    return resolved;
    }
};

module.exports = SongData;
