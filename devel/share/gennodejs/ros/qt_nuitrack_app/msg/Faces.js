// Auto-generated. Do not edit!

// (in-package qt_nuitrack_app.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FaceInfo = require('./FaceInfo.js');

//-----------------------------------------------------------

class Faces {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.faces = null;
    }
    else {
      if (initObj.hasOwnProperty('faces')) {
        this.faces = initObj.faces
      }
      else {
        this.faces = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Faces
    // Serialize message field [faces]
    // Serialize the length for message field [faces]
    bufferOffset = _serializer.uint32(obj.faces.length, buffer, bufferOffset);
    obj.faces.forEach((val) => {
      bufferOffset = FaceInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Faces
    let len;
    let data = new Faces(null);
    // Deserialize message field [faces]
    // Deserialize array length for message field [faces]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.faces = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.faces[i] = FaceInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.faces.forEach((val) => {
      length += FaceInfo.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qt_nuitrack_app/Faces';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '20dfb5db1e90db4dafd45bc663893505';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #std_msgs/Header header
    FaceInfo[] faces
    
    ================================================================================
    MSG: qt_nuitrack_app/FaceInfo
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
    const resolved = new Faces(null);
    if (msg.faces !== undefined) {
      resolved.faces = new Array(msg.faces.length);
      for (let i = 0; i < resolved.faces.length; ++i) {
        resolved.faces[i] = FaceInfo.Resolve(msg.faces[i]);
      }
    }
    else {
      resolved.faces = []
    }

    return resolved;
    }
};

module.exports = Faces;
