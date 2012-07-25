"""autogenerated by genpy from experiments/ExperimentParamsRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import experiments.msg
import patterngen.msg

class ExperimentParamsRequest(genpy.Message):
  _md5sum = "0891a3da32b0fce37a0d3e9662abde2d"
  _type = "experiments/ExperimentParamsRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """ExperimentSettings 	experiment
HomeSettings 		home
float64 			waitEntry
TriggerSettings 	triggerEntry
MoveSettings 		move
LasertrackSettings 	lasertrack
TriggerSettings 	triggerExit
SaveSettings 		save

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
bool 		tracking
string 		frameidOriginPosition # 'Plate' or 'Robot' or 'Fly'
string 		frameidOriginAngle # 'Plate' or 'Robot' or 'Fly'
float64 	distance
float64 	angle
string 		angleType # 'random' or 'constant'
float64 	speed
string 		speedType # 'random' or 'constant'
float64 	tolerance


================================================================================
MSG: experiments/MovePattern
string 				shape  # 'constant' or 'ramp' or 'circle' or 'square' or 'flylogo' or 'spiral'
float64 			hzPattern
float64 			hzPoint
int32 				count  # -1 means forever
geometry_msgs/Point size
float64             param


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: experiments/LasertrackSettings
bool 					enabled
patterngen/MsgPattern[] pattern_list
float64 				timeout


================================================================================
MSG: patterngen/MsgPattern
string                mode      # 'byshape' or 'bypoints'
string                shape     # 'constant' or 'square' or 'circle' or 'flylogo' or 'spiral' or 'ramp' or 'grid' or 'raster' or 'hilbert' or 'peano' or 'none'
string                frame_id  # The frame to which the pattern applies.
float64               hzPattern # Frequency of the pattern.
float64               hzPoint   # Frequency of points making up the pattern.
int32                 count     # How many times to output the pattern (-1 or N.inf means infinite).
geometry_msgs/Point[] points    # If mode=='bypoints', then this is the list of points to scan.
geometry_msgs/Point   size      # (x,y) dimensions.
bool				  preempt   # Should this message restart an in-progress pattern.
float64               param     # An extra shape-dependent parameter, if needed (hilbert->level, peano->level, spiral->pitch, raster->gridpitch).
 


================================================================================
MSG: experiments/SaveSettings
string filenamebase
bool arenastate
bool video
bool bag
bool onlyWhileTriggered


"""
  __slots__ = ['experiment','home','waitEntry','triggerEntry','move','lasertrack','triggerExit','save']
  _slot_types = ['experiments/ExperimentSettings','experiments/HomeSettings','float64','experiments/TriggerSettings','experiments/MoveSettings','experiments/LasertrackSettings','experiments/TriggerSettings','experiments/SaveSettings']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       experiment,home,waitEntry,triggerEntry,move,lasertrack,triggerExit,save

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
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
      if self.lasertrack is None:
        self.lasertrack = experiments.msg.LasertrackSettings()
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
      self.lasertrack = experiments.msg.LasertrackSettings()
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
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.experiment.description
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2iB6dB6d.pack(_x.experiment.maxTrials, _x.experiment.trial, _x.home.enabled, _x.home.x, _x.home.y, _x.home.speed, _x.home.tolerance, _x.home.timeout, _x.waitEntry, _x.triggerEntry.enabled, _x.triggerEntry.distanceMin, _x.triggerEntry.distanceMax, _x.triggerEntry.speedMin, _x.triggerEntry.speedMax, _x.triggerEntry.angleMin, _x.triggerEntry.angleMax))
      _x = self.triggerEntry.angleTest
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B2dB.pack(_x.triggerEntry.angleTestBilateral, _x.triggerEntry.timeHold, _x.triggerEntry.timeout, _x.move.enabled))
      _x = self.move.mode
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.move.relative.tracking))
      _x = self.move.relative.frameidOriginPosition
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.move.relative.frameidOriginAngle
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2d.pack(_x.move.relative.distance, _x.move.relative.angle))
      _x = self.move.relative.angleType
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_d.pack(self.move.relative.speed))
      _x = self.move.relative.speedType
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_d.pack(self.move.relative.tolerance))
      _x = self.move.pattern.shape
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2di5dB.pack(_x.move.pattern.hzPattern, _x.move.pattern.hzPoint, _x.move.pattern.count, _x.move.pattern.size.x, _x.move.pattern.size.y, _x.move.pattern.size.z, _x.move.pattern.param, _x.move.timeout, _x.lasertrack.enabled))
      length = len(self.lasertrack.pattern_list)
      buff.write(_struct_I.pack(length))
      for val1 in self.lasertrack.pattern_list:
        _x = val1.mode
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.shape
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_2di.pack(_x.hzPattern, _x.hzPoint, _x.count))
        length = len(val1.points)
        buff.write(_struct_I.pack(length))
        for val2 in val1.points:
          _x = val2
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v1 = val1.size
        _x = _v1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _x = val1
        buff.write(_struct_Bd.pack(_x.preempt, _x.param))
      _x = self
      buff.write(_struct_dB6d.pack(_x.lasertrack.timeout, _x.triggerExit.enabled, _x.triggerExit.distanceMin, _x.triggerExit.distanceMax, _x.triggerExit.speedMin, _x.triggerExit.speedMax, _x.triggerExit.angleMin, _x.triggerExit.angleMax))
      _x = self.triggerExit.angleTest
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B2d.pack(_x.triggerExit.angleTestBilateral, _x.triggerExit.timeHold, _x.triggerExit.timeout))
      _x = self.save.filenamebase
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_4B.pack(_x.save.arenastate, _x.save.video, _x.save.bag, _x.save.onlyWhileTriggered))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
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
      if self.lasertrack is None:
        self.lasertrack = experiments.msg.LasertrackSettings()
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
      if python3:
        self.experiment.description = str[start:end].decode('utf-8')
      else:
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
      if python3:
        self.triggerEntry.angleTest = str[start:end].decode('utf-8')
      else:
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
      if python3:
        self.move.mode = str[start:end].decode('utf-8')
      else:
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
      if python3:
        self.move.relative.frameidOriginPosition = str[start:end].decode('utf-8')
      else:
        self.move.relative.frameidOriginPosition = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.move.relative.frameidOriginAngle = str[start:end].decode('utf-8')
      else:
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
      if python3:
        self.move.relative.angleType = str[start:end].decode('utf-8')
      else:
        self.move.relative.angleType = str[start:end]
      start = end
      end += 8
      (self.move.relative.speed,) = _struct_d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.move.relative.speedType = str[start:end].decode('utf-8')
      else:
        self.move.relative.speedType = str[start:end]
      start = end
      end += 8
      (self.move.relative.tolerance,) = _struct_d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.move.pattern.shape = str[start:end].decode('utf-8')
      else:
        self.move.pattern.shape = str[start:end]
      _x = self
      start = end
      end += 61
      (_x.move.pattern.hzPattern, _x.move.pattern.hzPoint, _x.move.pattern.count, _x.move.pattern.size.x, _x.move.pattern.size.y, _x.move.pattern.size.z, _x.move.pattern.param, _x.move.timeout, _x.lasertrack.enabled,) = _struct_2di5dB.unpack(str[start:end])
      self.lasertrack.enabled = bool(self.lasertrack.enabled)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.lasertrack.pattern_list = []
      for i in range(0, length):
        val1 = patterngen.msg.MsgPattern()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.mode = str[start:end].decode('utf-8')
        else:
          val1.mode = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.shape = str[start:end].decode('utf-8')
        else:
          val1.shape = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.frame_id = str[start:end].decode('utf-8')
        else:
          val1.frame_id = str[start:end]
        _x = val1
        start = end
        end += 20
        (_x.hzPattern, _x.hzPoint, _x.count,) = _struct_2di.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.points = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Point()
          _x = val2
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          val1.points.append(val2)
        _v2 = val1.size
        _x = _v2
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _x = val1
        start = end
        end += 9
        (_x.preempt, _x.param,) = _struct_Bd.unpack(str[start:end])
        val1.preempt = bool(val1.preempt)
        self.lasertrack.pattern_list.append(val1)
      _x = self
      start = end
      end += 57
      (_x.lasertrack.timeout, _x.triggerExit.enabled, _x.triggerExit.distanceMin, _x.triggerExit.distanceMax, _x.triggerExit.speedMin, _x.triggerExit.speedMax, _x.triggerExit.angleMin, _x.triggerExit.angleMax,) = _struct_dB6d.unpack(str[start:end])
      self.triggerExit.enabled = bool(self.triggerExit.enabled)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.triggerExit.angleTest = str[start:end].decode('utf-8')
      else:
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
      if python3:
        self.save.filenamebase = str[start:end].decode('utf-8')
      else:
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
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.experiment.description
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2iB6dB6d.pack(_x.experiment.maxTrials, _x.experiment.trial, _x.home.enabled, _x.home.x, _x.home.y, _x.home.speed, _x.home.tolerance, _x.home.timeout, _x.waitEntry, _x.triggerEntry.enabled, _x.triggerEntry.distanceMin, _x.triggerEntry.distanceMax, _x.triggerEntry.speedMin, _x.triggerEntry.speedMax, _x.triggerEntry.angleMin, _x.triggerEntry.angleMax))
      _x = self.triggerEntry.angleTest
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B2dB.pack(_x.triggerEntry.angleTestBilateral, _x.triggerEntry.timeHold, _x.triggerEntry.timeout, _x.move.enabled))
      _x = self.move.mode
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.move.relative.tracking))
      _x = self.move.relative.frameidOriginPosition
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.move.relative.frameidOriginAngle
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2d.pack(_x.move.relative.distance, _x.move.relative.angle))
      _x = self.move.relative.angleType
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_d.pack(self.move.relative.speed))
      _x = self.move.relative.speedType
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_d.pack(self.move.relative.tolerance))
      _x = self.move.pattern.shape
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2di5dB.pack(_x.move.pattern.hzPattern, _x.move.pattern.hzPoint, _x.move.pattern.count, _x.move.pattern.size.x, _x.move.pattern.size.y, _x.move.pattern.size.z, _x.move.pattern.param, _x.move.timeout, _x.lasertrack.enabled))
      length = len(self.lasertrack.pattern_list)
      buff.write(_struct_I.pack(length))
      for val1 in self.lasertrack.pattern_list:
        _x = val1.mode
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.shape
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_2di.pack(_x.hzPattern, _x.hzPoint, _x.count))
        length = len(val1.points)
        buff.write(_struct_I.pack(length))
        for val2 in val1.points:
          _x = val2
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v3 = val1.size
        _x = _v3
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _x = val1
        buff.write(_struct_Bd.pack(_x.preempt, _x.param))
      _x = self
      buff.write(_struct_dB6d.pack(_x.lasertrack.timeout, _x.triggerExit.enabled, _x.triggerExit.distanceMin, _x.triggerExit.distanceMax, _x.triggerExit.speedMin, _x.triggerExit.speedMax, _x.triggerExit.angleMin, _x.triggerExit.angleMax))
      _x = self.triggerExit.angleTest
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B2d.pack(_x.triggerExit.angleTestBilateral, _x.triggerExit.timeHold, _x.triggerExit.timeout))
      _x = self.save.filenamebase
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_4B.pack(_x.save.arenastate, _x.save.video, _x.save.bag, _x.save.onlyWhileTriggered))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
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
      if self.lasertrack is None:
        self.lasertrack = experiments.msg.LasertrackSettings()
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
      if python3:
        self.experiment.description = str[start:end].decode('utf-8')
      else:
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
      if python3:
        self.triggerEntry.angleTest = str[start:end].decode('utf-8')
      else:
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
      if python3:
        self.move.mode = str[start:end].decode('utf-8')
      else:
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
      if python3:
        self.move.relative.frameidOriginPosition = str[start:end].decode('utf-8')
      else:
        self.move.relative.frameidOriginPosition = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.move.relative.frameidOriginAngle = str[start:end].decode('utf-8')
      else:
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
      if python3:
        self.move.relative.angleType = str[start:end].decode('utf-8')
      else:
        self.move.relative.angleType = str[start:end]
      start = end
      end += 8
      (self.move.relative.speed,) = _struct_d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.move.relative.speedType = str[start:end].decode('utf-8')
      else:
        self.move.relative.speedType = str[start:end]
      start = end
      end += 8
      (self.move.relative.tolerance,) = _struct_d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.move.pattern.shape = str[start:end].decode('utf-8')
      else:
        self.move.pattern.shape = str[start:end]
      _x = self
      start = end
      end += 61
      (_x.move.pattern.hzPattern, _x.move.pattern.hzPoint, _x.move.pattern.count, _x.move.pattern.size.x, _x.move.pattern.size.y, _x.move.pattern.size.z, _x.move.pattern.param, _x.move.timeout, _x.lasertrack.enabled,) = _struct_2di5dB.unpack(str[start:end])
      self.lasertrack.enabled = bool(self.lasertrack.enabled)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.lasertrack.pattern_list = []
      for i in range(0, length):
        val1 = patterngen.msg.MsgPattern()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.mode = str[start:end].decode('utf-8')
        else:
          val1.mode = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.shape = str[start:end].decode('utf-8')
        else:
          val1.shape = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.frame_id = str[start:end].decode('utf-8')
        else:
          val1.frame_id = str[start:end]
        _x = val1
        start = end
        end += 20
        (_x.hzPattern, _x.hzPoint, _x.count,) = _struct_2di.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.points = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Point()
          _x = val2
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          val1.points.append(val2)
        _v4 = val1.size
        _x = _v4
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _x = val1
        start = end
        end += 9
        (_x.preempt, _x.param,) = _struct_Bd.unpack(str[start:end])
        val1.preempt = bool(val1.preempt)
        self.lasertrack.pattern_list.append(val1)
      _x = self
      start = end
      end += 57
      (_x.lasertrack.timeout, _x.triggerExit.enabled, _x.triggerExit.distanceMin, _x.triggerExit.distanceMax, _x.triggerExit.speedMin, _x.triggerExit.speedMax, _x.triggerExit.angleMin, _x.triggerExit.angleMax,) = _struct_dB6d.unpack(str[start:end])
      self.triggerExit.enabled = bool(self.triggerExit.enabled)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.triggerExit.angleTest = str[start:end].decode('utf-8')
      else:
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
      if python3:
        self.save.filenamebase = str[start:end].decode('utf-8')
      else:
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
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_Bd = struct.Struct("<Bd")
_struct_2di5dB = struct.Struct("<2di5dB")
_struct_B = struct.Struct("<B")
_struct_d = struct.Struct("<d")
_struct_4B = struct.Struct("<4B")
_struct_B2dB = struct.Struct("<B2dB")
_struct_2d = struct.Struct("<2d")
_struct_B2d = struct.Struct("<B2d")
_struct_dB6d = struct.Struct("<dB6d")
_struct_3d = struct.Struct("<3d")
_struct_2di = struct.Struct("<2di")
_struct_2iB6dB6d = struct.Struct("<2iB6dB6d")
"""autogenerated by genpy from experiments/ExperimentParamsResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ExperimentParamsResponse(genpy.Message):
  _md5sum = "95e696a0d10686913abb262e0b4cbbcf"
  _type = "experiments/ExperimentParamsResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool 				succeeded



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

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
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
    :param buff: buffer, ``StringIO``
    """
    try:
      buff.write(_struct_B.pack(self.succeeded))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.succeeded,) = _struct_B.unpack(str[start:end])
      self.succeeded = bool(self.succeeded)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      buff.write(_struct_B.pack(self.succeeded))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.succeeded,) = _struct_B.unpack(str[start:end])
      self.succeeded = bool(self.succeeded)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
class ExperimentParams(object):
  _type          = 'experiments/ExperimentParams'
  _md5sum = '71a4fe056120e7ffb72be76bd9f6d505'
  _request_class  = ExperimentParamsRequest
  _response_class = ExperimentParamsResponse
