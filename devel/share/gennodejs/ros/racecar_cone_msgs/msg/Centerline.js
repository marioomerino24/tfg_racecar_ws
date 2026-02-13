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

class Centerline {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.samples = null;
      this.s = null;
      this.kappa = null;
      this.is_arclength_uniform = null;
      this.ds = null;
      this.ctrl_pts = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('samples')) {
        this.samples = initObj.samples
      }
      else {
        this.samples = [];
      }
      if (initObj.hasOwnProperty('s')) {
        this.s = initObj.s
      }
      else {
        this.s = [];
      }
      if (initObj.hasOwnProperty('kappa')) {
        this.kappa = initObj.kappa
      }
      else {
        this.kappa = [];
      }
      if (initObj.hasOwnProperty('is_arclength_uniform')) {
        this.is_arclength_uniform = initObj.is_arclength_uniform
      }
      else {
        this.is_arclength_uniform = false;
      }
      if (initObj.hasOwnProperty('ds')) {
        this.ds = initObj.ds
      }
      else {
        this.ds = 0.0;
      }
      if (initObj.hasOwnProperty('ctrl_pts')) {
        this.ctrl_pts = initObj.ctrl_pts
      }
      else {
        this.ctrl_pts = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Centerline
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [samples]
    // Serialize the length for message field [samples]
    bufferOffset = _serializer.uint32(obj.samples.length, buffer, bufferOffset);
    obj.samples.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [s]
    bufferOffset = _arraySerializer.float32(obj.s, buffer, bufferOffset, null);
    // Serialize message field [kappa]
    bufferOffset = _arraySerializer.float32(obj.kappa, buffer, bufferOffset, null);
    // Serialize message field [is_arclength_uniform]
    bufferOffset = _serializer.bool(obj.is_arclength_uniform, buffer, bufferOffset);
    // Serialize message field [ds]
    bufferOffset = _serializer.float32(obj.ds, buffer, bufferOffset);
    // Serialize message field [ctrl_pts]
    // Serialize the length for message field [ctrl_pts]
    bufferOffset = _serializer.uint32(obj.ctrl_pts.length, buffer, bufferOffset);
    obj.ctrl_pts.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Centerline
    let len;
    let data = new Centerline(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [samples]
    // Deserialize array length for message field [samples]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.samples = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.samples[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [s]
    data.s = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [kappa]
    data.kappa = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [is_arclength_uniform]
    data.is_arclength_uniform = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ds]
    data.ds = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ctrl_pts]
    // Deserialize array length for message field [ctrl_pts]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.ctrl_pts = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.ctrl_pts[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 24 * object.samples.length;
    length += 4 * object.s.length;
    length += 4 * object.kappa.length;
    length += 24 * object.ctrl_pts.length;
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'racecar_cone_msgs/Centerline';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3934b303e0826c747777abfdd52e228c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header                 # frame_id típico: "odom"
    
    # Muestras de la trayectoria c(s) remuestreadas
    geometry_msgs/Point[] samples          # tamaño N
    
    # Longitud de arco acumulada para cada sample (m)
    float32[] s                            # tamaño N, s[0]=0
    
    # Curvatura por muestra (1/m). Signo según convención (yaw izquierda +)
    float32[] kappa                        # tamaño N
    
    # Metadatos de muestreo
    bool is_arclength_uniform              # true si s es uniforme
    float32 ds                             # paso nominal en s (m), 0 si no aplica
    
    # (Opcional) puntos de control del ajuste (vacío si no aplica)
    geometry_msgs/Point[] ctrl_pts
    
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
    const resolved = new Centerline(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.samples !== undefined) {
      resolved.samples = new Array(msg.samples.length);
      for (let i = 0; i < resolved.samples.length; ++i) {
        resolved.samples[i] = geometry_msgs.msg.Point.Resolve(msg.samples[i]);
      }
    }
    else {
      resolved.samples = []
    }

    if (msg.s !== undefined) {
      resolved.s = msg.s;
    }
    else {
      resolved.s = []
    }

    if (msg.kappa !== undefined) {
      resolved.kappa = msg.kappa;
    }
    else {
      resolved.kappa = []
    }

    if (msg.is_arclength_uniform !== undefined) {
      resolved.is_arclength_uniform = msg.is_arclength_uniform;
    }
    else {
      resolved.is_arclength_uniform = false
    }

    if (msg.ds !== undefined) {
      resolved.ds = msg.ds;
    }
    else {
      resolved.ds = 0.0
    }

    if (msg.ctrl_pts !== undefined) {
      resolved.ctrl_pts = new Array(msg.ctrl_pts.length);
      for (let i = 0; i < resolved.ctrl_pts.length; ++i) {
        resolved.ctrl_pts[i] = geometry_msgs.msg.Point.Resolve(msg.ctrl_pts[i]);
      }
    }
    else {
      resolved.ctrl_pts = []
    }

    return resolved;
    }
};

module.exports = Centerline;
