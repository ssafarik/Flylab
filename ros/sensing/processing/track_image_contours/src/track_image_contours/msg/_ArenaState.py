"""autogenerated by genmsg_py from ArenaState.msg. Do not edit."""
import roslib.message
import struct

import track_image_contours.msg
import geometry_msgs.msg
import flystage.msg
import std_msgs.msg

class ArenaState(roslib.message.Message):
  _md5sum = "bd846af7c93b07705a6eca8d496d7107"
  _type = "track_image_contours/ArenaState"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """flystage/MsgFrameState robot
flystage/MsgFrameState[] flies
Stopped robot_stopped
Stopped fly_stopped

================================================================================
MSG: flystage/MsgFrameState
Header header
geometry_msgs/Pose pose
geometry_msgs/Twist velocity


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into it's linear and angular parts. 
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
================================================================================
MSG: track_image_contours/Stopped
bool stopped
"""
  __slots__ = ['robot','flies','robot_stopped','fly_stopped']
  _slot_types = ['flystage/MsgFrameState','flystage/MsgFrameState[]','track_image_contours/Stopped','track_image_contours/Stopped']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       robot,flies,robot_stopped,fly_stopped
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(ArenaState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.robot is None:
        self.robot = flystage.msg.MsgFrameState()
      if self.flies is None:
        self.flies = []
      if self.robot_stopped is None:
        self.robot_stopped = track_image_contours.msg.Stopped()
      if self.fly_stopped is None:
        self.fly_stopped = track_image_contours.msg.Stopped()
    else:
      self.robot = flystage.msg.MsgFrameState()
      self.flies = []
      self.robot_stopped = track_image_contours.msg.Stopped()
      self.fly_stopped = track_image_contours.msg.Stopped()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.robot.header.seq, _x.robot.header.stamp.secs, _x.robot.header.stamp.nsecs))
      _x = self.robot.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_13d.pack(_x.robot.pose.position.x, _x.robot.pose.position.y, _x.robot.pose.position.z, _x.robot.pose.orientation.x, _x.robot.pose.orientation.y, _x.robot.pose.orientation.z, _x.robot.pose.orientation.w, _x.robot.velocity.linear.x, _x.robot.velocity.linear.y, _x.robot.velocity.linear.z, _x.robot.velocity.angular.x, _x.robot.velocity.angular.y, _x.robot.velocity.angular.z))
      length = len(self.flies)
      buff.write(_struct_I.pack(length))
      for val1 in self.flies:
        _v1 = val1.header
        buff.write(_struct_I.pack(_v1.seq))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v3 = val1.pose
        _v4 = _v3.position
        _x = _v4
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v5 = _v3.orientation
        _x = _v5
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        _v6 = val1.velocity
        _v7 = _v6.linear
        _x = _v7
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v8 = _v6.angular
        _x = _v8
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_struct_2B.pack(_x.robot_stopped.stopped, _x.fly_stopped.stopped))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.robot is None:
        self.robot = flystage.msg.MsgFrameState()
      if self.robot_stopped is None:
        self.robot_stopped = track_image_contours.msg.Stopped()
      if self.fly_stopped is None:
        self.fly_stopped = track_image_contours.msg.Stopped()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.robot.header.seq, _x.robot.header.stamp.secs, _x.robot.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.robot.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 104
      (_x.robot.pose.position.x, _x.robot.pose.position.y, _x.robot.pose.position.z, _x.robot.pose.orientation.x, _x.robot.pose.orientation.y, _x.robot.pose.orientation.z, _x.robot.pose.orientation.w, _x.robot.velocity.linear.x, _x.robot.velocity.linear.y, _x.robot.velocity.linear.z, _x.robot.velocity.angular.x, _x.robot.velocity.angular.y, _x.robot.velocity.angular.z,) = _struct_13d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.flies = []
      for i in range(0, length):
        val1 = flystage.msg.MsgFrameState()
        _v9 = val1.header
        start = end
        end += 4
        (_v9.seq,) = _struct_I.unpack(str[start:end])
        _v10 = _v9.stamp
        _x = _v10
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        _v9.frame_id = str[start:end]
        _v11 = val1.pose
        _v12 = _v11.position
        _x = _v12
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v13 = _v11.orientation
        _x = _v13
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        _v14 = val1.velocity
        _v15 = _v14.linear
        _x = _v15
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v16 = _v14.angular
        _x = _v16
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.flies.append(val1)
      _x = self
      start = end
      end += 2
      (_x.robot_stopped.stopped, _x.fly_stopped.stopped,) = _struct_2B.unpack(str[start:end])
      self.robot_stopped.stopped = bool(self.robot_stopped.stopped)
      self.fly_stopped.stopped = bool(self.fly_stopped.stopped)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.robot.header.seq, _x.robot.header.stamp.secs, _x.robot.header.stamp.nsecs))
      _x = self.robot.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_13d.pack(_x.robot.pose.position.x, _x.robot.pose.position.y, _x.robot.pose.position.z, _x.robot.pose.orientation.x, _x.robot.pose.orientation.y, _x.robot.pose.orientation.z, _x.robot.pose.orientation.w, _x.robot.velocity.linear.x, _x.robot.velocity.linear.y, _x.robot.velocity.linear.z, _x.robot.velocity.angular.x, _x.robot.velocity.angular.y, _x.robot.velocity.angular.z))
      length = len(self.flies)
      buff.write(_struct_I.pack(length))
      for val1 in self.flies:
        _v17 = val1.header
        buff.write(_struct_I.pack(_v17.seq))
        _v18 = _v17.stamp
        _x = _v18
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v17.frame_id
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v19 = val1.pose
        _v20 = _v19.position
        _x = _v20
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v21 = _v19.orientation
        _x = _v21
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        _v22 = val1.velocity
        _v23 = _v22.linear
        _x = _v23
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v24 = _v22.angular
        _x = _v24
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_struct_2B.pack(_x.robot_stopped.stopped, _x.fly_stopped.stopped))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.robot is None:
        self.robot = flystage.msg.MsgFrameState()
      if self.robot_stopped is None:
        self.robot_stopped = track_image_contours.msg.Stopped()
      if self.fly_stopped is None:
        self.fly_stopped = track_image_contours.msg.Stopped()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.robot.header.seq, _x.robot.header.stamp.secs, _x.robot.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.robot.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 104
      (_x.robot.pose.position.x, _x.robot.pose.position.y, _x.robot.pose.position.z, _x.robot.pose.orientation.x, _x.robot.pose.orientation.y, _x.robot.pose.orientation.z, _x.robot.pose.orientation.w, _x.robot.velocity.linear.x, _x.robot.velocity.linear.y, _x.robot.velocity.linear.z, _x.robot.velocity.angular.x, _x.robot.velocity.angular.y, _x.robot.velocity.angular.z,) = _struct_13d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.flies = []
      for i in range(0, length):
        val1 = flystage.msg.MsgFrameState()
        _v25 = val1.header
        start = end
        end += 4
        (_v25.seq,) = _struct_I.unpack(str[start:end])
        _v26 = _v25.stamp
        _x = _v26
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        _v25.frame_id = str[start:end]
        _v27 = val1.pose
        _v28 = _v27.position
        _x = _v28
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v29 = _v27.orientation
        _x = _v29
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        _v30 = val1.velocity
        _v31 = _v30.linear
        _x = _v31
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v32 = _v30.angular
        _x = _v32
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.flies.append(val1)
      _x = self
      start = end
      end += 2
      (_x.robot_stopped.stopped, _x.fly_stopped.stopped,) = _struct_2B.unpack(str[start:end])
      self.robot_stopped.stopped = bool(self.robot_stopped.stopped)
      self.fly_stopped.stopped = bool(self.fly_stopped.stopped)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_13d = struct.Struct("<13d")
_struct_3I = struct.Struct("<3I")
_struct_2B = struct.Struct("<2B")
_struct_4d = struct.Struct("<4d")
_struct_2I = struct.Struct("<2I")
_struct_3d = struct.Struct("<3d")
