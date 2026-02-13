// Auto-generated. Do not edit!

// (in-package racecar_cone_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TrackMetrics {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.coverage = null;
      this.std_width = null;
      this.max_kappa = null;
      this.integral_kappa_sq = null;
      this.latency_ms = null;
      this.cones_raw_count = null;
      this.pairs_count = null;
      this.midpoints_count = null;
      this.notes = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('coverage')) {
        this.coverage = initObj.coverage
      }
      else {
        this.coverage = 0.0;
      }
      if (initObj.hasOwnProperty('std_width')) {
        this.std_width = initObj.std_width
      }
      else {
        this.std_width = 0.0;
      }
      if (initObj.hasOwnProperty('max_kappa')) {
        this.max_kappa = initObj.max_kappa
      }
      else {
        this.max_kappa = 0.0;
      }
      if (initObj.hasOwnProperty('integral_kappa_sq')) {
        this.integral_kappa_sq = initObj.integral_kappa_sq
      }
      else {
        this.integral_kappa_sq = 0.0;
      }
      if (initObj.hasOwnProperty('latency_ms')) {
        this.latency_ms = initObj.latency_ms
      }
      else {
        this.latency_ms = 0.0;
      }
      if (initObj.hasOwnProperty('cones_raw_count')) {
        this.cones_raw_count = initObj.cones_raw_count
      }
      else {
        this.cones_raw_count = 0;
      }
      if (initObj.hasOwnProperty('pairs_count')) {
        this.pairs_count = initObj.pairs_count
      }
      else {
        this.pairs_count = 0;
      }
      if (initObj.hasOwnProperty('midpoints_count')) {
        this.midpoints_count = initObj.midpoints_count
      }
      else {
        this.midpoints_count = 0;
      }
      if (initObj.hasOwnProperty('notes')) {
        this.notes = initObj.notes
      }
      else {
        this.notes = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackMetrics
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [coverage]
    bufferOffset = _serializer.float32(obj.coverage, buffer, bufferOffset);
    // Serialize message field [std_width]
    bufferOffset = _serializer.float32(obj.std_width, buffer, bufferOffset);
    // Serialize message field [max_kappa]
    bufferOffset = _serializer.float32(obj.max_kappa, buffer, bufferOffset);
    // Serialize message field [integral_kappa_sq]
    bufferOffset = _serializer.float32(obj.integral_kappa_sq, buffer, bufferOffset);
    // Serialize message field [latency_ms]
    bufferOffset = _serializer.float32(obj.latency_ms, buffer, bufferOffset);
    // Serialize message field [cones_raw_count]
    bufferOffset = _serializer.uint32(obj.cones_raw_count, buffer, bufferOffset);
    // Serialize message field [pairs_count]
    bufferOffset = _serializer.uint32(obj.pairs_count, buffer, bufferOffset);
    // Serialize message field [midpoints_count]
    bufferOffset = _serializer.uint32(obj.midpoints_count, buffer, bufferOffset);
    // Serialize message field [notes]
    bufferOffset = _serializer.string(obj.notes, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackMetrics
    let len;
    let data = new TrackMetrics(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [coverage]
    data.coverage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [std_width]
    data.std_width = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_kappa]
    data.max_kappa = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [integral_kappa_sq]
    data.integral_kappa_sq = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [latency_ms]
    data.latency_ms = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cones_raw_count]
    data.cones_raw_count = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pairs_count]
    data.pairs_count = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [midpoints_count]
    data.midpoints_count = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [notes]
    data.notes = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.notes.length;
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'racecar_cone_msgs/TrackMetrics';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9f81acf08fbcb042f5be8a05aaee3bf6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    # Cobertura de emparejado (0..1)
    float32 coverage
    
    # Estadística de anchos
    float32 std_width                      # desviación estándar de width (m)
    
    # Curvatura y suavidad
    float32 max_kappa                      # max |kappa| (1/m)
    float32 integral_kappa_sq              # ∫ kappa^2 ds (1/m)
    
    # Rendimiento
    float32 latency_ms                     # latencia media del ciclo (ms)
    
    # Contadores
    uint32 cones_raw_count
    uint32 pairs_count
    uint32 midpoints_count
    
    # Notas/diagnóstico
    string notes
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrackMetrics(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.coverage !== undefined) {
      resolved.coverage = msg.coverage;
    }
    else {
      resolved.coverage = 0.0
    }

    if (msg.std_width !== undefined) {
      resolved.std_width = msg.std_width;
    }
    else {
      resolved.std_width = 0.0
    }

    if (msg.max_kappa !== undefined) {
      resolved.max_kappa = msg.max_kappa;
    }
    else {
      resolved.max_kappa = 0.0
    }

    if (msg.integral_kappa_sq !== undefined) {
      resolved.integral_kappa_sq = msg.integral_kappa_sq;
    }
    else {
      resolved.integral_kappa_sq = 0.0
    }

    if (msg.latency_ms !== undefined) {
      resolved.latency_ms = msg.latency_ms;
    }
    else {
      resolved.latency_ms = 0.0
    }

    if (msg.cones_raw_count !== undefined) {
      resolved.cones_raw_count = msg.cones_raw_count;
    }
    else {
      resolved.cones_raw_count = 0
    }

    if (msg.pairs_count !== undefined) {
      resolved.pairs_count = msg.pairs_count;
    }
    else {
      resolved.pairs_count = 0
    }

    if (msg.midpoints_count !== undefined) {
      resolved.midpoints_count = msg.midpoints_count;
    }
    else {
      resolved.midpoints_count = 0
    }

    if (msg.notes !== undefined) {
      resolved.notes = msg.notes;
    }
    else {
      resolved.notes = ''
    }

    return resolved;
    }
};

module.exports = TrackMetrics;
