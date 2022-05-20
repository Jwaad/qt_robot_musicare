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

class FaceInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.gender = null;
      this.age_years = null;
      this.age_type = null;
      this.emotion_neutral = null;
      this.emotion_angry = null;
      this.emotion_happy = null;
      this.emotion_surprise = null;
      this.rectangle = null;
      this.left_eye = null;
      this.right_eye = null;
      this.angles = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('gender')) {
        this.gender = initObj.gender
      }
      else {
        this.gender = '';
      }
      if (initObj.hasOwnProperty('age_years')) {
        this.age_years = initObj.age_years
      }
      else {
        this.age_years = 0;
      }
      if (initObj.hasOwnProperty('age_type')) {
        this.age_type = initObj.age_type
      }
      else {
        this.age_type = '';
      }
      if (initObj.hasOwnProperty('emotion_neutral')) {
        this.emotion_neutral = initObj.emotion_neutral
      }
      else {
        this.emotion_neutral = 0.0;
      }
      if (initObj.hasOwnProperty('emotion_angry')) {
        this.emotion_angry = initObj.emotion_angry
      }
      else {
        this.emotion_angry = 0.0;
      }
      if (initObj.hasOwnProperty('emotion_happy')) {
        this.emotion_happy = initObj.emotion_happy
      }
      else {
        this.emotion_happy = 0.0;
      }
      if (initObj.hasOwnProperty('emotion_surprise')) {
        this.emotion_surprise = initObj.emotion_surprise
      }
      else {
        this.emotion_surprise = 0.0;
      }
      if (initObj.hasOwnProperty('rectangle')) {
        this.rectangle = initObj.rectangle
      }
      else {
        this.rectangle = [];
      }
      if (initObj.hasOwnProperty('left_eye')) {
        this.left_eye = initObj.left_eye
      }
      else {
        this.left_eye = [];
      }
      if (initObj.hasOwnProperty('right_eye')) {
        this.right_eye = initObj.right_eye
      }
      else {
        this.right_eye = [];
      }
      if (initObj.hasOwnProperty('angles')) {
        this.angles = initObj.angles
      }
      else {
        this.angles = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FaceInfo
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [gender]
    bufferOffset = _serializer.string(obj.gender, buffer, bufferOffset);
    // Serialize message field [age_years]
    bufferOffset = _serializer.int32(obj.age_years, buffer, bufferOffset);
    // Serialize message field [age_type]
    bufferOffset = _serializer.string(obj.age_type, buffer, bufferOffset);
    // Serialize message field [emotion_neutral]
    bufferOffset = _serializer.float64(obj.emotion_neutral, buffer, bufferOffset);
    // Serialize message field [emotion_angry]
    bufferOffset = _serializer.float64(obj.emotion_angry, buffer, bufferOffset);
    // Serialize message field [emotion_happy]
    bufferOffset = _serializer.float64(obj.emotion_happy, buffer, bufferOffset);
    // Serialize message field [emotion_surprise]
    bufferOffset = _serializer.float64(obj.emotion_surprise, buffer, bufferOffset);
    // Serialize message field [rectangle]
    bufferOffset = _arraySerializer.float64(obj.rectangle, buffer, bufferOffset, null);
    // Serialize message field [left_eye]
    bufferOffset = _arraySerializer.float64(obj.left_eye, buffer, bufferOffset, null);
    // Serialize message field [right_eye]
    bufferOffset = _arraySerializer.float64(obj.right_eye, buffer, bufferOffset, null);
    // Serialize message field [angles]
    bufferOffset = _arraySerializer.float64(obj.angles, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FaceInfo
    let len;
    let data = new FaceInfo(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [gender]
    data.gender = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [age_years]
    data.age_years = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [age_type]
    data.age_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [emotion_neutral]
    data.emotion_neutral = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [emotion_angry]
    data.emotion_angry = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [emotion_happy]
    data.emotion_happy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [emotion_surprise]
    data.emotion_surprise = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rectangle]
    data.rectangle = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [left_eye]
    data.left_eye = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [right_eye]
    data.right_eye = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [angles]
    data.angles = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.gender);
    length += _getByteLength(object.age_type);
    length += 8 * object.rectangle.length;
    length += 8 * object.left_eye.length;
    length += 8 * object.right_eye.length;
    length += 8 * object.angles.length;
    return length + 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qt_nuitrack_app/FaceInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b210720ecbe10f5a0a7a3911214f766a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #std_msgs/Header header
    int32 id
    string gender
    int32 age_years
    string age_type
    float64 emotion_neutral
    float64 emotion_angry
    float64 emotion_happy
    float64 emotion_surprise
    #normalized screen coordinates of a face rectangle in the image (x, y, w, h)
    float64[] rectangle
    #normalized coordinates of the center of a person’s left eye (x,y)
    float64[] left_eye
    #normalized coordinates of the center of a person’s right eye (x,y)
    float64[] right_eye
    #face orientation angles in degrees (yaw, pitch, roll)
    float64[] angles
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FaceInfo(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.gender !== undefined) {
      resolved.gender = msg.gender;
    }
    else {
      resolved.gender = ''
    }

    if (msg.age_years !== undefined) {
      resolved.age_years = msg.age_years;
    }
    else {
      resolved.age_years = 0
    }

    if (msg.age_type !== undefined) {
      resolved.age_type = msg.age_type;
    }
    else {
      resolved.age_type = ''
    }

    if (msg.emotion_neutral !== undefined) {
      resolved.emotion_neutral = msg.emotion_neutral;
    }
    else {
      resolved.emotion_neutral = 0.0
    }

    if (msg.emotion_angry !== undefined) {
      resolved.emotion_angry = msg.emotion_angry;
    }
    else {
      resolved.emotion_angry = 0.0
    }

    if (msg.emotion_happy !== undefined) {
      resolved.emotion_happy = msg.emotion_happy;
    }
    else {
      resolved.emotion_happy = 0.0
    }

    if (msg.emotion_surprise !== undefined) {
      resolved.emotion_surprise = msg.emotion_surprise;
    }
    else {
      resolved.emotion_surprise = 0.0
    }

    if (msg.rectangle !== undefined) {
      resolved.rectangle = msg.rectangle;
    }
    else {
      resolved.rectangle = []
    }

    if (msg.left_eye !== undefined) {
      resolved.left_eye = msg.left_eye;
    }
    else {
      resolved.left_eye = []
    }

    if (msg.right_eye !== undefined) {
      resolved.right_eye = msg.right_eye;
    }
    else {
      resolved.right_eye = []
    }

    if (msg.angles !== undefined) {
      resolved.angles = msg.angles;
    }
    else {
      resolved.angles = []
    }

    return resolved;
    }
};

module.exports = FaceInfo;
