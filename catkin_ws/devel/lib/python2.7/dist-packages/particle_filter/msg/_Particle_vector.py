# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from particle_filter/Particle_vector.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import particle_filter.msg

class Particle_vector(genpy.Message):
  _md5sum = "1011eaf19eb1e2338cfeb6b0c73b6b4f"
  _type = "particle_filter/Particle_vector"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """Particle[] particles

================================================================================
MSG: particle_filter/Particle
Pose pose
float64 weight

================================================================================
MSG: particle_filter/Pose
float64 x
float64 y
float64 theta

"""
  __slots__ = ['particles']
  _slot_types = ['particle_filter/Particle[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       particles

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Particle_vector, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.particles is None:
        self.particles = []
    else:
      self.particles = []

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
      length = len(self.particles)
      buff.write(_struct_I.pack(length))
      for val1 in self.particles:
        _v1 = val1.pose
        _x = _v1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.theta))
        buff.write(_struct_d.pack(val1.weight))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.particles is None:
        self.particles = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.particles = []
      for i in range(0, length):
        val1 = particle_filter.msg.Particle()
        _v2 = val1.pose
        _x = _v2
        start = end
        end += 24
        (_x.x, _x.y, _x.theta,) = _struct_3d.unpack(str[start:end])
        start = end
        end += 8
        (val1.weight,) = _struct_d.unpack(str[start:end])
        self.particles.append(val1)
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
      length = len(self.particles)
      buff.write(_struct_I.pack(length))
      for val1 in self.particles:
        _v3 = val1.pose
        _x = _v3
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.theta))
        buff.write(_struct_d.pack(val1.weight))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.particles is None:
        self.particles = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.particles = []
      for i in range(0, length):
        val1 = particle_filter.msg.Particle()
        _v4 = val1.pose
        _x = _v4
        start = end
        end += 24
        (_x.x, _x.y, _x.theta,) = _struct_3d.unpack(str[start:end])
        start = end
        end += 8
        (val1.weight,) = _struct_d.unpack(str[start:end])
        self.particles.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_d = struct.Struct("<d")
_struct_3d = struct.Struct("<3d")
