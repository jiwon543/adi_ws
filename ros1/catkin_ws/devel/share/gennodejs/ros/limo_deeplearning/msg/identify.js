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

class identify {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.results = null;
      this.classes = null;
      this.area = null;
      this.position = null;
      this.acc = null;
      this.image_number = null;
    }
    else {
      if (initObj.hasOwnProperty('results')) {
        this.results = initObj.results
      }
      else {
        this.results = '';
      }
      if (initObj.hasOwnProperty('classes')) {
        this.classes = initObj.classes
      }
      else {
        this.classes = 0;
      }
      if (initObj.hasOwnProperty('area')) {
        this.area = initObj.area
      }
      else {
        this.area = 0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = 0.0;
      }
      if (initObj.hasOwnProperty('image_number')) {
        this.image_number = initObj.image_number
      }
      else {
        this.image_number = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type identify
    // Serialize message field [results]
    bufferOffset = _serializer.string(obj.results, buffer, bufferOffset);
    // Serialize message field [classes]
    bufferOffset = _serializer.int8(obj.classes, buffer, bufferOffset);
    // Serialize message field [area]
    bufferOffset = _serializer.int32(obj.area, buffer, bufferOffset);
    // Check that the constant length array field [position] has the right length
    if (obj.position.length !== 4) {
      throw new Error('Unable to serialize array field position - length must be 4')
    }
    // Serialize message field [position]
    bufferOffset = _arraySerializer.float32(obj.position, buffer, bufferOffset, 4);
    // Serialize message field [acc]
    bufferOffset = _serializer.float32(obj.acc, buffer, bufferOffset);
    // Serialize message field [image_number]
    bufferOffset = _serializer.int32(obj.image_number, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type identify
    let len;
    let data = new identify(null);
    // Deserialize message field [results]
    data.results = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [classes]
    data.classes = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [area]
    data.area = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [acc]
    data.acc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [image_number]
    data.image_number = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.results);
    return length + 33;
  }

  static datatype() {
    // Returns string type for a message object
    return 'limo_deeplearning/identify';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f6053d8d260d6714b1da39ebe6fbf86b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string results
    int8 classes
    int32 area
    float32[4] position
    float32 acc
    int32 image_number
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new identify(null);
    if (msg.results !== undefined) {
      resolved.results = msg.results;
    }
    else {
      resolved.results = ''
    }

    if (msg.classes !== undefined) {
      resolved.classes = msg.classes;
    }
    else {
      resolved.classes = 0
    }

    if (msg.area !== undefined) {
      resolved.area = msg.area;
    }
    else {
      resolved.area = 0
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = new Array(4).fill(0)
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = 0.0
    }

    if (msg.image_number !== undefined) {
      resolved.image_number = msg.image_number;
    }
    else {
      resolved.image_number = 0
    }

    return resolved;
    }
};

module.exports = identify;
