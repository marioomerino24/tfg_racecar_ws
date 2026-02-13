// Auto-generated. Do not edit!

// (in-package racecar_cone_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ConePair = require('./ConePair.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ConePairArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.pairs = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('pairs')) {
        this.pairs = initObj.pairs
      }
      else {
        this.pairs = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConePairArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [pairs]
    // Serialize the length for message field [pairs]
    bufferOffset = _serializer.uint32(obj.pairs.length, buffer, bufferOffset);
    obj.pairs.forEach((val) => {
      bufferOffset = ConePair.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConePairArray
    let len;
    let data = new ConePairArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [pairs]
    // Deserialize array length for message field [pairs]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pairs = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pairs[i] = ConePair.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.pairs.forEach((val) => {
      length += ConePair.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'racecar_cone_msgs/ConePairArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fbdccabe23e08b5ded654d3910450cdc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    racecar_cone_msgs/ConePair[] pairs
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: racecar_cone_msgs/ConePair
    std_msgs/Header header
    racecar_cone_msgs/Cone left
    racecar_cone_msgs/Cone right
    geometry_msgs/Point midpoint
    float32 width                   # ||right - left||
    
    ================================================================================
    MSG: racecar_cone_msgs/Cone
    # Detección de un cono (frame fijo: odom/map)
    std_msgs/Header header
    geometry_msgs/Point p           # posición (z=0 si 2D)
    # Covarianza plana 2x2 (x,y) aplanada (row-major): [xx, xy, yx, yy]
    float32[4] cov_xy
    
    # Lateralidad
    uint8 UNKNOWN=0
    uint8 LEFT=1
    uint8 RIGHT=2
    uint8 side
    
    float32 confidence              # [0..1]
    string source                   # "lidar", "sim", etc.
    int32 id                        # opcional (tracking)
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConePairArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.pairs !== undefined) {
      resolved.pairs = new Array(msg.pairs.length);
      for (let i = 0; i < resolved.pairs.length; ++i) {
        resolved.pairs[i] = ConePair.Resolve(msg.pairs[i]);
      }
    }
    else {
      resolved.pairs = []
    }

    return resolved;
    }
};

module.exports = ConePairArray;
