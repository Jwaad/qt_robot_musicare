# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from qt_nuitrack_app/Hands.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import qt_nuitrack_app.msg

class Hands(genpy.Message):
  _md5sum = "633d76b336567f0906335e6cf0195299"
  _type = "qt_nuitrack_app/Hands"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """#std_msgs/Header header
HandInfo[] hands

================================================================================
MSG: qt_nuitrack_app/HandInfo
#std_msgs/Header header
int32 id
#The normalized projective (x, y) coordinate of the right hand
float32[] right_projection
#The (x,y,z) coordinate of the right hand in the world system.
float32[] right_real
bool right_click
int32 right_pressure
#The normalized projective (x, y) coordinate of the left hand
float32[] left_projection
#The (x,y,z) coordinate of the left hand in the world system.
float32[] left_real
bool left_click
int32 left_pressure
"""
  __slots__ = ['hands']
  _slot_types = ['qt_nuitrack_app/HandInfo[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       hands

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Hands, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.hands is None:
        self.hands = []
    else:
      self.hands = []

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
      length = len(self.hands)
      buff.write(_struct_I.pack(length))
      for val1 in self.hands:
        _x = val1.id
        buff.write(_get_struct_i().pack(_x))
        length = len(val1.right_projection)
        buff.write(_struct_I.pack(length))
        pattern = '<%sf'%length
        buff.write(struct.pack(pattern, *val1.right_projection))
        length = len(val1.right_real)
        buff.write(_struct_I.pack(length))
        pattern = '<%sf'%length
        buff.write(struct.pack(pattern, *val1.right_real))
        _x = val1
        buff.write(_get_struct_Bi().pack(_x.right_click, _x.right_pressure))
        length = len(val1.left_projection)
        buff.write(_struct_I.pack(length))
        pattern = '<%sf'%length
        buff.write(struct.pack(pattern, *val1.left_projection))
        length = len(val1.left_real)
        buff.write(_struct_I.pack(length))
        pattern = '<%sf'%length
        buff.write(struct.pack(pattern, *val1.left_real))
        _x = val1
        buff.write(_get_struct_Bi().pack(_x.left_click, _x.left_pressure))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.hands is None:
        self.hands = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.hands = []
      for i in range(0, length):
        val1 = qt_nuitrack_app.msg.HandInfo()
        start = end
        end += 4
        (val1.id,) = _get_struct_i().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sf'%length
        start = end
        end += struct.calcsize(pattern)
        val1.right_projection = struct.unpack(pattern, str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sf'%length
        start = end
        end += struct.calcsize(pattern)
        val1.right_real = struct.unpack(pattern, str[start:end])
        _x = val1
        start = end
        end += 5
        (_x.right_click, _x.right_pressure,) = _get_struct_Bi().unpack(str[start:end])
        val1.right_click = bool(val1.right_click)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sf'%length
        start = end
        end += struct.calcsize(pattern)
        val1.left_projection = struct.unpack(pattern, str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sf'%length
        start = end
        end += struct.calcsize(pattern)
        val1.left_real = struct.unpack(pattern, str[start:end])
        _x = val1
        start = end
        end += 5
        (_x.left_click, _x.left_pressure,) = _get_struct_Bi().unpack(str[start:end])
        val1.left_click = bool(val1.left_click)
        self.hands.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.hands)
      buff.write(_struct_I.pack(length))
      for val1 in self.hands:
        _x = val1.id
        buff.write(_get_struct_i().pack(_x))
        length = len(val1.right_projection)
        buff.write(_struct_I.pack(length))
        pattern = '<%sf'%length
        buff.write(val1.right_projection.tostring())
        length = len(val1.right_real)
        buff.write(_struct_I.pack(length))
        pattern = '<%sf'%length
        buff.write(val1.right_real.tostring())
        _x = val1
        buff.write(_get_struct_Bi().pack(_x.right_click, _x.right_pressure))
        length = len(val1.left_projection)
        buff.write(_struct_I.pack(length))
        pattern = '<%sf'%length
        buff.write(val1.left_projection.tostring())
        length = len(val1.left_real)
        buff.write(_struct_I.pack(length))
        pattern = '<%sf'%length
        buff.write(val1.left_real.tostring())
        _x = val1
        buff.write(_get_struct_Bi().pack(_x.left_click, _x.left_pressure))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.hands is None:
        self.hands = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.hands = []
      for i in range(0, length):
        val1 = qt_nuitrack_app.msg.HandInfo()
        start = end
        end += 4
        (val1.id,) = _get_struct_i().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sf'%length
        start = end
        end += struct.calcsize(pattern)
        val1.right_projection = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sf'%length
        start = end
        end += struct.calcsize(pattern)
        val1.right_real = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
        _x = val1
        start = end
        end += 5
        (_x.right_click, _x.right_pressure,) = _get_struct_Bi().unpack(str[start:end])
        val1.right_click = bool(val1.right_click)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sf'%length
        start = end
        end += struct.calcsize(pattern)
        val1.left_projection = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sf'%length
        start = end
        end += struct.calcsize(pattern)
        val1.left_real = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
        _x = val1
        start = end
        end += 5
        (_x.left_click, _x.left_pressure,) = _get_struct_Bi().unpack(str[start:end])
        val1.left_click = bool(val1.left_click)
        self.hands.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_Bi = None
def _get_struct_Bi():
    global _struct_Bi
    if _struct_Bi is None:
        _struct_Bi = struct.Struct("<Bi")
    return _struct_Bi
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
