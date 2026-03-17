// Auto-generated. Do not edit!

// (in-package object_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Object = require('./Object.js');

//-----------------------------------------------------------

class ObjectArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Objects = null;
    }
    else {
      if (initObj.hasOwnProperty('Objects')) {
        this.Objects = initObj.Objects
      }
      else {
        this.Objects = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjectArray
    // Serialize message field [Objects]
    // Serialize the length for message field [Objects]
    bufferOffset = _serializer.uint32(obj.Objects.length, buffer, bufferOffset);
    obj.Objects.forEach((val) => {
      bufferOffset = Object.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectArray
    let len;
    let data = new ObjectArray(null);
    // Deserialize message field [Objects]
    // Deserialize array length for message field [Objects]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.Objects = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.Objects[i] = Object.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.Objects.forEach((val) => {
      length += Object.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_msgs/ObjectArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a82955a978c818e847a2c852e8536db8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Object[] Objects
    ================================================================================
    MSG: object_msgs/Object
    string class_name
    uint32[] xmin_ymin_xmax_ymax
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObjectArray(null);
    if (msg.Objects !== undefined) {
      resolved.Objects = new Array(msg.Objects.length);
      for (let i = 0; i < resolved.Objects.length; ++i) {
        resolved.Objects[i] = Object.Resolve(msg.Objects[i]);
      }
    }
    else {
      resolved.Objects = []
    }

    return resolved;
    }
};

module.exports = ObjectArray;
