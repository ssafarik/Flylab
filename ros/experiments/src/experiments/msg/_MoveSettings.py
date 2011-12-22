"""autogenerated by genmsg_py from MoveSettings.msg. Do not edit."""
import roslib.message
import struct


class MoveSettings(roslib.message.Message):
  _md5sum = "8a3c810f3c1f2cb5fb317a984814d92e"
  _type = "experiments/MoveSettings"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool enabled
bool tracking
string frameidOriginPosition # 'Plate' or 'Robot' or 'Fly'
string frameidOriginAngle # 'Plate' or 'Robot' or 'Fly'
float64 distance
float64 angle
string angleType # 'random' or 'constant'
float64 velocity
string velocityType # 'random' or 'constant'
float64 tolerance
float64 timeout


"""
  __slots__ = ['enabled','tracking','frameidOriginPosition','frameidOriginAngle','distance','angle','angleType','velocity','velocityType','tolerance','timeout']
  _slot_types = ['bool','bool','string','string','float64','float64','string','float64','string','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       enabled,tracking,frameidOriginPosition,frameidOriginAngle,distance,angle,angleType,velocity,velocityType,tolerance,timeout
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(MoveSettings, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.enabled is None:
        self.enabled = False
      if self.tracking is None:
        self.tracking = False
      if self.frameidOriginPosition is None:
        self.frameidOriginPosition = ''
      if self.frameidOriginAngle is None:
        self.frameidOriginAngle = ''
      if self.distance is None:
        self.distance = 0.
      if self.angle is None:
        self.angle = 0.
      if self.angleType is None:
        self.angleType = ''
      if self.velocity is None:
        self.velocity = 0.
      if self.velocityType is None:
        self.velocityType = ''
      if self.tolerance is None:
        self.tolerance = 0.
      if self.timeout is None:
        self.timeout = 0.
    else:
      self.enabled = False
      self.tracking = False
      self.frameidOriginPosition = ''
      self.frameidOriginAngle = ''
      self.distance = 0.
      self.angle = 0.
      self.angleType = ''
      self.velocity = 0.
      self.velocityType = ''
      self.tolerance = 0.
      self.timeout = 0.

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
      buff.write(_struct_2B.pack(_x.enabled, _x.tracking))
      _x = self.frameidOriginPosition
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.frameidOriginAngle
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2d.pack(_x.distance, _x.angle))
      _x = self.angleType
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_d.pack(self.velocity))
      _x = self.velocityType
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2d.pack(_x.tolerance, _x.timeout))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 2
      (_x.enabled, _x.tracking,) = _struct_2B.unpack(str[start:end])
      self.enabled = bool(self.enabled)
      self.tracking = bool(self.tracking)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.frameidOriginPosition = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.frameidOriginAngle = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.distance, _x.angle,) = _struct_2d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.angleType = str[start:end]
      start = end
      end += 8
      (self.velocity,) = _struct_d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.velocityType = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.tolerance, _x.timeout,) = _struct_2d.unpack(str[start:end])
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
      buff.write(_struct_2B.pack(_x.enabled, _x.tracking))
      _x = self.frameidOriginPosition
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.frameidOriginAngle
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2d.pack(_x.distance, _x.angle))
      _x = self.angleType
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_d.pack(self.velocity))
      _x = self.velocityType
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2d.pack(_x.tolerance, _x.timeout))
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
      end = 0
      _x = self
      start = end
      end += 2
      (_x.enabled, _x.tracking,) = _struct_2B.unpack(str[start:end])
      self.enabled = bool(self.enabled)
      self.tracking = bool(self.tracking)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.frameidOriginPosition = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.frameidOriginAngle = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.distance, _x.angle,) = _struct_2d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.angleType = str[start:end]
      start = end
      end += 8
      (self.velocity,) = _struct_d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.velocityType = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.tolerance, _x.timeout,) = _struct_2d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_2d = struct.Struct("<2d")
_struct_d = struct.Struct("<d")
_struct_2B = struct.Struct("<2B")
