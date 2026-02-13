// Auto-generated. Do not edit!

// (in-package racecar_cone_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Cone = require('./Cone.js');
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ConePair {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.left = null;
      this.right = null;
      this.midpoint = null;
      this.width = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('left')) {
        this.left = initObj.left
      }
      else {
        this.left = new Cone();
      }
      if (initObj.hasOwnProperty('right')) {
        this.right = initObj.right
      }
      else {
        this.right = new Cone();
      }
      if (initObj.hasOwnProperty('midpoint')) {
        this.midpoint = initObj.midpoint
      }
      else {
        this.midpoint = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConePair
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [left]
    bufferOffset = Cone.serialize(obj.left, buffer, bufferOffset);
    // Serialize message field [right]
    bufferOffset = Cone.serialize(obj.right, buffer, bufferOffset);
    // Serialize message field [midpoint]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.midpoint, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.float32(obj.width, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConePair
    let len;
    let data = new ConePair(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [left]
    data.left = Cone.deserialize(buffer, bufferOffset);
    // Deserialize message field [right]
    data.right = Cone.deserialize(buffer, bufferOffset);
    // Deserialize message field [midpoint]
    data.midpoint = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += Cone.getMessageSize(object.left);
    length += Cone.getMessageSize(object.right);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'racecar_cone_msgs/ConePair';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e86328cf0c5c9f6e1d43796e78dbb005';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    racecar_cone_msgs/Cone left
    racecar_cone_msgs/Cone right
    geometry_msgs/Point midpoint
    float32 width                   # ||right - left||
    
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
    const resolved = new ConePair(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.left !== undefined) {
      resolved.left = Cone.Resolve(msg.left)
    }
    else {
      resolved.left = new Cone()
    }

    if (msg.right !== undefined) {
      resolved.right = Cone.Resolve(msg.right)
    }
    else {
      resolved.right = new Cone()
    }

    if (msg.midpoint !== undefined) {
      resolved.midpoint = geometry_msgs.msg.Point.Resolve(msg.midpoint)
    }
    else {
      resolved.midpoint = new geometry_msgs.msg.Point()
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0.0
    }

    return resolved;
    }
};

module.exports = ConePair;
