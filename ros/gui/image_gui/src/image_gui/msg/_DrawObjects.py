"""autogenerated by genmsg_py from DrawObjects.msg. Do not edit."""
import roslib.message
import struct

import image_gui.msg

class DrawObjects(roslib.message.Message):
  _md5sum = "09167aa6575b6803d68a4405eaa911e3"
  _type = "image_gui/DrawObjects"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool show_all
bool hide_all
DrawObject[] draw_object_list

================================================================================
MSG: image_gui/DrawObject
bool show
CvPoint object_center
CvLine[] line_list
CvCircle[] circle_list
================================================================================
MSG: image_gui/CvPoint
int32 x
int32 y

================================================================================
MSG: image_gui/CvLine
CvPoint point1
CvPoint point2
CvColor color
int32 thickness
int32 lineType
int32 shift
================================================================================
MSG: image_gui/CvColor
float64 red
float64 green
float64 blue
================================================================================
MSG: image_gui/CvCircle
CvPoint center
int32 radius
CvColor color
int32 thickness
int32 lineType
int32 shift
"""
  __slots__ = ['show_all','hide_all','draw_object_list']
  _slot_types = ['bool','bool','image_gui/DrawObject[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       show_all,hide_all,draw_object_list
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(DrawObjects, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.show_all is None:
        self.show_all = False
      if self.hide_all is None:
        self.hide_all = False
      if self.draw_object_list is None:
        self.draw_object_list = []
    else:
      self.show_all = False
      self.hide_all = False
      self.draw_object_list = []

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
      buff.write(_struct_2B.pack(_x.show_all, _x.hide_all))
      length = len(self.draw_object_list)
      buff.write(_struct_I.pack(length))
      for val1 in self.draw_object_list:
        buff.write(_struct_B.pack(val1.show))
        _v1 = val1.object_center
        _x = _v1
        buff.write(_struct_2i.pack(_x.x, _x.y))
        length = len(val1.line_list)
        buff.write(_struct_I.pack(length))
        for val2 in val1.line_list:
          _v2 = val2.point1
          _x = _v2
          buff.write(_struct_2i.pack(_x.x, _x.y))
          _v3 = val2.point2
          _x = _v3
          buff.write(_struct_2i.pack(_x.x, _x.y))
          _v4 = val2.color
          _x = _v4
          buff.write(_struct_3d.pack(_x.red, _x.green, _x.blue))
          _x = val2
          buff.write(_struct_3i.pack(_x.thickness, _x.lineType, _x.shift))
        length = len(val1.circle_list)
        buff.write(_struct_I.pack(length))
        for val2 in val1.circle_list:
          _v5 = val2.center
          _x = _v5
          buff.write(_struct_2i.pack(_x.x, _x.y))
          buff.write(_struct_i.pack(val2.radius))
          _v6 = val2.color
          _x = _v6
          buff.write(_struct_3d.pack(_x.red, _x.green, _x.blue))
          _x = val2
          buff.write(_struct_3i.pack(_x.thickness, _x.lineType, _x.shift))
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
      (_x.show_all, _x.hide_all,) = _struct_2B.unpack(str[start:end])
      self.show_all = bool(self.show_all)
      self.hide_all = bool(self.hide_all)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.draw_object_list = []
      for i in range(0, length):
        val1 = image_gui.msg.DrawObject()
        start = end
        end += 1
        (val1.show,) = _struct_B.unpack(str[start:end])
        val1.show = bool(val1.show)
        _v7 = val1.object_center
        _x = _v7
        start = end
        end += 8
        (_x.x, _x.y,) = _struct_2i.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.line_list = []
        for i in range(0, length):
          val2 = image_gui.msg.CvLine()
          _v8 = val2.point1
          _x = _v8
          start = end
          end += 8
          (_x.x, _x.y,) = _struct_2i.unpack(str[start:end])
          _v9 = val2.point2
          _x = _v9
          start = end
          end += 8
          (_x.x, _x.y,) = _struct_2i.unpack(str[start:end])
          _v10 = val2.color
          _x = _v10
          start = end
          end += 24
          (_x.red, _x.green, _x.blue,) = _struct_3d.unpack(str[start:end])
          _x = val2
          start = end
          end += 12
          (_x.thickness, _x.lineType, _x.shift,) = _struct_3i.unpack(str[start:end])
          val1.line_list.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.circle_list = []
        for i in range(0, length):
          val2 = image_gui.msg.CvCircle()
          _v11 = val2.center
          _x = _v11
          start = end
          end += 8
          (_x.x, _x.y,) = _struct_2i.unpack(str[start:end])
          start = end
          end += 4
          (val2.radius,) = _struct_i.unpack(str[start:end])
          _v12 = val2.color
          _x = _v12
          start = end
          end += 24
          (_x.red, _x.green, _x.blue,) = _struct_3d.unpack(str[start:end])
          _x = val2
          start = end
          end += 12
          (_x.thickness, _x.lineType, _x.shift,) = _struct_3i.unpack(str[start:end])
          val1.circle_list.append(val2)
        self.draw_object_list.append(val1)
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
      buff.write(_struct_2B.pack(_x.show_all, _x.hide_all))
      length = len(self.draw_object_list)
      buff.write(_struct_I.pack(length))
      for val1 in self.draw_object_list:
        buff.write(_struct_B.pack(val1.show))
        _v13 = val1.object_center
        _x = _v13
        buff.write(_struct_2i.pack(_x.x, _x.y))
        length = len(val1.line_list)
        buff.write(_struct_I.pack(length))
        for val2 in val1.line_list:
          _v14 = val2.point1
          _x = _v14
          buff.write(_struct_2i.pack(_x.x, _x.y))
          _v15 = val2.point2
          _x = _v15
          buff.write(_struct_2i.pack(_x.x, _x.y))
          _v16 = val2.color
          _x = _v16
          buff.write(_struct_3d.pack(_x.red, _x.green, _x.blue))
          _x = val2
          buff.write(_struct_3i.pack(_x.thickness, _x.lineType, _x.shift))
        length = len(val1.circle_list)
        buff.write(_struct_I.pack(length))
        for val2 in val1.circle_list:
          _v17 = val2.center
          _x = _v17
          buff.write(_struct_2i.pack(_x.x, _x.y))
          buff.write(_struct_i.pack(val2.radius))
          _v18 = val2.color
          _x = _v18
          buff.write(_struct_3d.pack(_x.red, _x.green, _x.blue))
          _x = val2
          buff.write(_struct_3i.pack(_x.thickness, _x.lineType, _x.shift))
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
      (_x.show_all, _x.hide_all,) = _struct_2B.unpack(str[start:end])
      self.show_all = bool(self.show_all)
      self.hide_all = bool(self.hide_all)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.draw_object_list = []
      for i in range(0, length):
        val1 = image_gui.msg.DrawObject()
        start = end
        end += 1
        (val1.show,) = _struct_B.unpack(str[start:end])
        val1.show = bool(val1.show)
        _v19 = val1.object_center
        _x = _v19
        start = end
        end += 8
        (_x.x, _x.y,) = _struct_2i.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.line_list = []
        for i in range(0, length):
          val2 = image_gui.msg.CvLine()
          _v20 = val2.point1
          _x = _v20
          start = end
          end += 8
          (_x.x, _x.y,) = _struct_2i.unpack(str[start:end])
          _v21 = val2.point2
          _x = _v21
          start = end
          end += 8
          (_x.x, _x.y,) = _struct_2i.unpack(str[start:end])
          _v22 = val2.color
          _x = _v22
          start = end
          end += 24
          (_x.red, _x.green, _x.blue,) = _struct_3d.unpack(str[start:end])
          _x = val2
          start = end
          end += 12
          (_x.thickness, _x.lineType, _x.shift,) = _struct_3i.unpack(str[start:end])
          val1.line_list.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.circle_list = []
        for i in range(0, length):
          val2 = image_gui.msg.CvCircle()
          _v23 = val2.center
          _x = _v23
          start = end
          end += 8
          (_x.x, _x.y,) = _struct_2i.unpack(str[start:end])
          start = end
          end += 4
          (val2.radius,) = _struct_i.unpack(str[start:end])
          _v24 = val2.color
          _x = _v24
          start = end
          end += 24
          (_x.red, _x.green, _x.blue,) = _struct_3d.unpack(str[start:end])
          _x = val2
          start = end
          end += 12
          (_x.thickness, _x.lineType, _x.shift,) = _struct_3i.unpack(str[start:end])
          val1.circle_list.append(val2)
        self.draw_object_list.append(val1)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_B = struct.Struct("<B")
_struct_i = struct.Struct("<i")
_struct_3i = struct.Struct("<3i")
_struct_2B = struct.Struct("<2B")
_struct_2i = struct.Struct("<2i")
_struct_3d = struct.Struct("<3d")
