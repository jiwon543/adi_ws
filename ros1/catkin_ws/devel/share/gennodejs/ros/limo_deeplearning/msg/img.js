// Auto-generated. Do not edit!

// (in-package limo_deeplearning.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class img {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.img = null;
    }
    else {
      if (initObj.hasOwnProperty('img')) {
        this.img = initObj.img
      }
      else {
        this.img = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type img
    // Serialize message field [img]
    bufferOffset = _arraySerializer.int16(obj.img, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type img
    let len;
    let data = new img(null);
    // Deserialize message field [img]
    data.img = _arrayDeserializer.int16(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 2 * object.img.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'limo_deeplearning/img';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '06c52dd5e482fac9219b8508a9f9cc99';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16[] img
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new img(null);
    if (msg.img !== undefined) {
      resolved.img = msg.img;
    }
    else {
      resolved.img = []
    }

    return resolved;
    }
};

module.exports = img;
