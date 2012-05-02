"""autogenerated by genmsg_py from ExperimentParamsRequest.msg. Do not edit."""
import roslib.message
import struct

import experiments.msg

class ExperimentParamsRequest(roslib.message.Message):
  _md5sum = "1e95b5f2c80621a002d46555834ab68e"
  _type = "experiments/ExperimentParamsRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """ExperimentSettings experiment
HomeSettings home
float64 waitEntry
TriggerSettings triggerEntry
MoveSettings move
TriggerSettings triggerExit
SaveSettings save

================================================================================
MSG: experiments/ExperimentSettings
string description
int32 maxTrials
int32 trial 


================================================================================
MSG: experiments/HomeSettings
bool enabled
float64 x
float64 y
float64 speed
float64 tolerance
float64 timeout



================================================================================
MSG: experiments/TriggerSettings
bool enabled
float64 distanceMin
float64 distanceMax
float64 speedMin
float64 speedMax
float64 angleMin
float64 angleMax
string  angleTest
bool    angleTestBilateral
float64 timeHold
float64 timeout



================================================================================
MSG: experiments/MoveSettings
bool enabled
string mode  # 'pattern' or 'relative'
MoveRelative relative
MovePattern pattern
float64 timeout


================================================================================
MSG: experiments/MoveRelative
bool tracking
string frameidOriginPosition # 'Plate' or 'Robot' or 'Fly'
string frameidOriginAngle # 'Plate' or 'Robot' or 'Fly'
float64 distance
float64 angle
string angleType # 'random' or 'constant'
float64 speed
string speedType # 'random' or 'constant'
float64 tolerance


================================================================================
MSG: experiments/MovePattern
string shape  # 'constant' or 'ramp' or 'circle' or 'square' or 'flylogo' or 'spiral'
float64 hzPattern
float64 hzPoint
int32 count  # -1 means forever
float64 radius


================================================================================
MSG: experiments/SaveSettings
string filenamebase
bool arenastate
bool video
bool bag
bool onlyWhileTriggered


"""
  __slots__ = ['experiment','home','waitEntry','triggerEntry','move','triggerExit','save']
  _slot_types = ['experiments/ExperimentSettings','experiments/HomeSettings','float64','experiments/TriggerSettings','experiments/MoveSettings','experiments/TriggerSettings','experiments/SaveSettings']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       experiment,home,waitEntry,triggerEntry,move,triggerExit,save
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(ExperimentParamsRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.experiment is None:
        self.experiment = experiments.msg.ExperimentSettings()
      if self.home is None:
        self.home = experiments.msg.HomeSettings()
      if self.waitEntry is None:
        self.waitEntry = 0.
      if self.triggerEntry is None:
        self.triggerEntry = experiments.msg.TriggerSettings()
      if self.move is None:
        self.move = experiments.msg.MoveSettings()
      if self.triggerExit is None:
        self.triggerExit = experiments.msg.TriggerSettings()
      if self.save is None:
        self.save = experiments.msg.SaveSettings()
    else:
      self.experiment = experiments.msg.ExperimentSettings()
      self.home = experiments.msg.HomeSettings()
      self.waitEntry = 0.
      self.triggerEntry = experiments.msg.TriggerSettings()
      self.move = experiments.msg.MoveSettings()
      self.triggerExit = experiments.msg.TriggerSettings()
      self.save = experiments.msg.SaveSettings()

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
      _x = self.experiment.description
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2iB6dB6d.pack(_x.experiment.maxTrials, _x.experiment.trial, _x.home.enabled, _x.home.x, _x.home.y, _x.home.speed, _x.home.tolerance, _x.home.timeout, _x.waitEntry, _x.triggerEntry.enabled, _x.triggerEntry.distanceMin, _x.triggerEntry.distanceMax, _x.triggerEntry.speedMin, _x.triggerEntry.speedMax, _x.triggerEntry.angleMin, _x.triggerEntry.angleMax))
      _x = self.triggerEntry.angleTest
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B2dB.pack(_x.triggerEntry.angleTestBilateral, _x.triggerEntry.timeHold, _x.triggerEntry.timeout, _x.move.enabled))
      _x = self.move.mode
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.move.relative.tracking))
      _x = self.move.relative.frameidOriginPosition
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.move.relative.frameidOriginAngle
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2d.pack(_x.move.relative.distance, _x.move.relative.angle))
      _x = self.move.relative.angleType
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_d.pack(self.move.relative.speed))
      _x = self.move.relative.speedType
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_d.pack(self.move.relative.tolerance))
      _x = self.move.pattern.shape
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2di2dB6d.pack(_x.move.pattern.hzPattern, _x.move.pattern.hzPoint, _x.move.pattern.count, _x.move.pattern.radius, _x.move.timeout, _x.triggerExit.enabled, _x.triggerExit.distanceMin, _x.triggerExit.distanceMax, _x.triggerExit.speedMin, _x.triggerExit.speedMax, _x.triggerExit.angleMin, _x.triggerExit.angleMax))
      _x = self.triggerExit.angleTest
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B2d.pack(_x.triggerExit.angleTestBilateral, _x.triggerExit.timeHold, _x.triggerExit.timeout))
      _x = self.save.filenamebase
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_4B.pack(_x.save.arenastate, _x.save.video, _x.save.bag, _x.save.onlyWhileTriggered))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.experiment is None:
        self.experiment = experiments.msg.ExperimentSettings()
      if self.home is None:
        self.home = experiments.msg.HomeSettings()
      if self.triggerEntry is None:
        self.triggerEntry = experiments.msg.TriggerSettings()
      if self.move is None:
        self.move = experiments.msg.MoveSettings()
      if self.triggerExit is None:
        self.triggerExit = experiments.msg.TriggerSettings()
      if self.save is None:
        self.save = experiments.msg.SaveSettings()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.experiment.description = str[start:end]
      _x = self
      start = end
      end += 106
      (_x.experiment.maxTrials, _x.experiment.trial, _x.home.enabled, _x.home.x, _x.home.y, _x.home.speed, _x.home.tolerance, _x.home.timeout, _x.waitEntry, _x.triggerEntry.enabled, _x.triggerEntry.distanceMin, _x.triggerEntry.distanceMax, _x.triggerEntry.speedMin, _x.triggerEntry.speedMax, _x.triggerEntry.angleMin, _x.triggerEntry.angleMax,) = _struct_2iB6dB6d.unpack(str[start:end])
      self.home.enabled = bool(self.home.enabled)
      self.triggerEntry.enabled = bool(self.triggerEntry.enabled)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.triggerEntry.angleTest = str[start:end]
      _x = self
      start = end
      end += 18
      (_x.triggerEntry.angleTestBilateral, _x.triggerEntry.timeHold, _x.triggerEntry.timeout, _x.move.enabled,) = _struct_B2dB.unpack(str[start:end])
      self.triggerEntry.angleTestBilateral = bool(self.triggerEntry.angleTestBilateral)
      self.move.enabled = bool(self.move.enabled)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.move.mode = str[start:end]
      start = end
      end += 1
      (self.move.relative.tracking,) = _struct_B.unpack(str[start:end])
      self.move.relative.tracking = bool(self.move.relative.tracking)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.move.relative.frameidOriginPosition = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.move.relative.frameidOriginAngle = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.move.relative.distance, _x.move.relative.angle,) = _struct_2d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.move.relative.angleType = str[start:end]
      start = end
      end += 8
      (self.move.relative.speed,) = _struct_d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.move.relative.speedType = str[start:end]
      start = end
      end += 8
      (self.move.relative.tolerance,) = _struct_d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.move.pattern.shape = str[start:end]
      _x = self
      start = end
      end += 85
      (_x.move.pattern.hzPattern, _x.move.pattern.hzPoint, _x.move.pattern.count, _x.move.pattern.radius, _x.move.timeout, _x.triggerExit.enabled, _x.triggerExit.distanceMin, _x.triggerExit.distanceMax, _x.triggerExit.speedMin, _x.triggerExit.speedMax, _x.triggerExit.angleMin, _x.triggerExit.angleMax,) = _struct_2di2dB6d.unpack(str[start:end])
      self.triggerExit.enabled = bool(self.triggerExit.enabled)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.triggerExit.angleTest = str[start:end]
      _x = self
      start = end
      end += 17
      (_x.triggerExit.angleTestBilateral, _x.triggerExit.timeHold, _x.triggerExit.timeout,) = _struct_B2d.unpack(str[start:end])
      self.triggerExit.angleTestBilateral = bool(self.triggerExit.angleTestBilateral)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.save.filenamebase = str[start:end]
      _x = self
      start = end
      end += 4
      (_x.save.arenastate, _x.save.video, _x.save.bag, _x.save.onlyWhileTriggered,) = _struct_4B.unpack(str[start:end])
      self.save.arenastate = bool(self.save.arenastate)
      self.save.video = bool(self.save.video)
      self.save.bag = bool(self.save.bag)
      self.save.onlyWhileTriggered = bool(self.save.onlyWhileTriggered)
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
      _x = self.experiment.description
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2iB6dB6d.pack(_x.experiment.maxTrials, _x.experiment.trial, _x.home.enabled, _x.home.x, _x.home.y, _x.home.speed, _x.home.tolerance, _x.home.timeout, _x.waitEntry, _x.triggerEntry.enabled, _x.triggerEntry.distanceMin, _x.triggerEntry.distanceMax, _x.triggerEntry.speedMin, _x.triggerEntry.speedMax, _x.triggerEntry.angleMin, _x.triggerEntry.angleMax))
      _x = self.triggerEntry.angleTest
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B2dB.pack(_x.triggerEntry.angleTestBilateral, _x.triggerEntry.timeHold, _x.triggerEntry.timeout, _x.move.enabled))
      _x = self.move.mode
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.move.relative.tracking))
      _x = self.move.relative.frameidOriginPosition
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.move.relative.frameidOriginAngle
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2d.pack(_x.move.relative.distance, _x.move.relative.angle))
      _x = self.move.relative.angleType
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_d.pack(self.move.relative.speed))
      _x = self.move.relative.speedType
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_d.pack(self.move.relative.tolerance))
      _x = self.move.pattern.shape
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2di2dB6d.pack(_x.move.pattern.hzPattern, _x.move.pattern.hzPoint, _x.move.pattern.count, _x.move.pattern.radius, _x.move.timeout, _x.triggerExit.enabled, _x.triggerExit.distanceMin, _x.triggerExit.distanceMax, _x.triggerExit.speedMin, _x.triggerExit.speedMax, _x.triggerExit.angleMin, _x.triggerExit.angleMax))
      _x = self.triggerExit.angleTest
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B2d.pack(_x.triggerExit.angleTestBilateral, _x.triggerExit.timeHold, _x.triggerExit.timeout))
      _x = self.save.filenamebase
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_4B.pack(_x.save.arenastate, _x.save.video, _x.save.bag, _x.save.onlyWhileTriggered))
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
      if self.experiment is None:
        self.experiment = experiments.msg.ExperimentSettings()
      if self.home is None:
        self.home = experiments.msg.HomeSettings()
      if self.triggerEntry is None:
        self.triggerEntry = experiments.msg.TriggerSettings()
      if self.move is None:
        self.move = experiments.msg.MoveSettings()
      if self.triggerExit is None:
        self.triggerExit = experiments.msg.TriggerSettings()
      if self.save is None:
        self.save = experiments.msg.SaveSettings()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.experiment.description = str[start:end]
      _x = self
      start = end
      end += 106
      (_x.experiment.maxTrials, _x.experiment.trial, _x.home.enabled, _x.home.x, _x.home.y, _x.home.speed, _x.home.tolerance, _x.home.timeout, _x.waitEntry, _x.triggerEntry.enabled, _x.triggerEntry.distanceMin, _x.triggerEntry.distanceMax, _x.triggerEntry.speedMin, _x.triggerEntry.speedMax, _x.triggerEntry.angleMin, _x.triggerEntry.angleMax,) = _struct_2iB6dB6d.unpack(str[start:end])
      self.home.enabled = bool(self.home.enabled)
      self.triggerEntry.enabled = bool(self.triggerEntry.enabled)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.triggerEntry.angleTest = str[start:end]
      _x = self
      start = end
      end += 18
      (_x.triggerEntry.angleTestBilateral, _x.triggerEntry.timeHold, _x.triggerEntry.timeout, _x.move.enabled,) = _struct_B2dB.unpack(str[start:end])
      self.triggerEntry.angleTestBilateral = bool(self.triggerEntry.angleTestBilateral)
      self.move.enabled = bool(self.move.enabled)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.move.mode = str[start:end]
      start = end
      end += 1
      (self.move.relative.tracking,) = _struct_B.unpack(str[start:end])
      self.move.relative.tracking = bool(self.move.relative.tracking)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.move.relative.frameidOriginPosition = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.move.relative.frameidOriginAngle = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.move.relative.distance, _x.move.relative.angle,) = _struct_2d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.move.relative.angleType = str[start:end]
      start = end
      end += 8
      (self.move.relative.speed,) = _struct_d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.move.relative.speedType = str[start:end]
      start = end
      end += 8
      (self.move.relative.tolerance,) = _struct_d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.move.pattern.shape = str[start:end]
      _x = self
      start = end
      end += 85
      (_x.move.pattern.hzPattern, _x.move.pattern.hzPoint, _x.move.pattern.count, _x.move.pattern.radius, _x.move.timeout, _x.triggerExit.enabled, _x.triggerExit.distanceMin, _x.triggerExit.distanceMax, _x.triggerExit.speedMin, _x.triggerExit.speedMax, _x.triggerExit.angleMin, _x.triggerExit.angleMax,) = _struct_2di2dB6d.unpack(str[start:end])
      self.triggerExit.enabled = bool(self.triggerExit.enabled)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.triggerExit.angleTest = str[start:end]
      _x = self
      start = end
      end += 17
      (_x.triggerExit.angleTestBilateral, _x.triggerExit.timeHold, _x.triggerExit.timeout,) = _struct_B2d.unpack(str[start:end])
      self.triggerExit.angleTestBilateral = bool(self.triggerExit.angleTestBilateral)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.save.filenamebase = str[start:end]
      _x = self
      start = end
      end += 4
      (_x.save.arenastate, _x.save.video, _x.save.bag, _x.save.onlyWhileTriggered,) = _struct_4B.unpack(str[start:end])
      self.save.arenastate = bool(self.save.arenastate)
      self.save.video = bool(self.save.video)
      self.save.bag = bool(self.save.bag)
      self.save.onlyWhileTriggered = bool(self.save.onlyWhileTriggered)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_B = struct.Struct("<B")
_struct_d = struct.Struct("<d")
_struct_B2dB = struct.Struct("<B2dB")
_struct_2d = struct.Struct("<2d")
_struct_B2d = struct.Struct("<B2d")
_struct_2di2dB6d = struct.Struct("<2di2dB6d")
_struct_4B = struct.Struct("<4B")
_struct_2iB6dB6d = struct.Struct("<2iB6dB6d")
"""autogenerated by genmsg_py from ExperimentParamsResponse.msg. Do not edit."""
import roslib.message
import struct


class ExperimentParamsResponse(roslib.message.Message):
  _md5sum = "95e696a0d10686913abb262e0b4cbbcf"
  _type = "experiments/ExperimentParamsResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool succeeded



"""
  __slots__ = ['succeeded']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       succeeded
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(ExperimentParamsResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.succeeded is None:
        self.succeeded = False
    else:
      self.succeeded = False

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
      buff.write(_struct_B.pack(self.succeeded))
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
      start = end
      end += 1
      (self.succeeded,) = _struct_B.unpack(str[start:end])
      self.succeeded = bool(self.succeeded)
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
      buff.write(_struct_B.pack(self.succeeded))
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
      start = end
      end += 1
      (self.succeeded,) = _struct_B.unpack(str[start:end])
      self.succeeded = bool(self.succeeded)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_B = struct.Struct("<B")
class ExperimentParams(roslib.message.ServiceDefinition):
  _type          = 'experiments/ExperimentParams'
  _md5sum = '2e4da8739f4b9829d90d50823130dccd'
  _request_class  = ExperimentParamsRequest
  _response_class = ExperimentParamsResponse