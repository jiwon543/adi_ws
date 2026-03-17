// Auto-generated. Do not edit!

// (in-package object_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Object {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.class_name = null;
      this.xmin_ymin_xmax_ymax = null;
    }
    else {
      if (initObj.hasOwnProperty('class_name')) {
        this.class_name = initObj.class_name
      }
      else {
        this.class_name = '';
      }
      if (initObj.hasOwnProperty('xmin_ymin_xmax_ymax')) {
        this.xmin_ymin_xmax_ymax = initObj.xmin_ymin_xmax_ymax
      }
      else {
        this.xmin_ymin_xmax_ymax = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Object
    // Serialize message field [class_name]
    bufferOffset = _serializer.string(obj.class_name, buffer, bufferOffset);
    // Serialize message field [xmin_ymin_xmax_ymax]
    bufferOffset = _arraySerializer.uint32(obj.xmin_ymin_xmax_ymax, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Object
    let len;
    let data = new Object(null);
    // Deserialize message field [class_name]
    data.class_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [xmin_ymin_xmax_ymax]
    data.xmin_ymin_xmax_ymax = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.class_name);
    length += 4 * object.xmin_ymin_xmax_ymax.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_msgs/Object';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f9beacd4cdb37d3951c7c33ecebfabd1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string class_name
    uint32[] xmin_ymin_xmax_ymax
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Object(null);
    if (msg.class_name !== undefined) {
      resolved.class_name = msg.class_name;
    }
    else {
      resolved.class_name = ''
    }

    if (msg.xmin_ymin_xmax_ymax !== undefined) {
      resolved.xmin_ymin_xmax_ymax = msg.xmin_ymin_xmax_ymax;
    }
    else {
      resolved.xmin_ymin_xmax_ymax = []
    }

    return resolved;
    }
};

module.exports = Object;
