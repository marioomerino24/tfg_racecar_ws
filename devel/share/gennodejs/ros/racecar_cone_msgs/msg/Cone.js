// Auto-generated. Do not edit!

// (in-package racecar_cone_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Cone {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.p = null;
      this.cov_xy = null;
      this.side = null;
      this.confidence = null;
      this.source = null;
      this.id = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('p')) {
        this.p = initObj.p
      }
      else {
        this.p = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('cov_xy')) {
        this.cov_xy = initObj.cov_xy
      }
      else {
        this.cov_xy = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('side')) {
        this.side = initObj.side
      }
      else {
        this.side = 0;
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('source')) {
        this.source = initObj.source
      }
      else {
        this.source = '';
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Cone
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [p]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.p, buffer, bufferOffset);
    // Check that the constant length array field [cov_xy] has the right length
    if (obj.cov_xy.length !== 4) {
      throw new Error('Unable to serialize array field cov_xy - length must be 4')
    }
    // Serialize message field [cov_xy]
    bufferOffset = _arraySerializer.float32(obj.cov_xy, buffer, bufferOffset, 4);
    // Serialize message field [side]
    bufferOffset = _serializer.uint8(obj.side, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    // Serialize message field [source]
    bufferOffset = _serializer.string(obj.source, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Cone
    let len;
    let data = new Cone(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [p]
    data.p = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [cov_xy]
    data.cov_xy = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [side]
    data.side = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [source]
    data.source = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.source.length;
    return length + 53;
  }

  static datatype() {
    // Returns string type for a message object
    return 'racecar_cone_msgs/Cone';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac79e60f67f210d41f97d0cd883ed6b5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Cone(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.p !== undefined) {
      resolved.p = geometry_msgs.msg.Point.Resolve(msg.p)
    }
    else {
      resolved.p = new geometry_msgs.msg.Point()
    }

    if (msg.cov_xy !== undefined) {
      resolved.cov_xy = msg.cov_xy;
    }
    else {
      resolved.cov_xy = new Array(4).fill(0)
    }

    if (msg.side !== undefined) {
      resolved.side = msg.side;
    }
    else {
      resolved.side = 0
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.source !== undefined) {
      resolved.source = msg.source;
    }
    else {
      resolved.source = ''
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    return resolved;
    }
};

// Constants for message
Cone.Constants = {
  UNKNOWN: 0,
  LEFT: 1,
  RIGHT: 2,
}

module.exports = Cone;
