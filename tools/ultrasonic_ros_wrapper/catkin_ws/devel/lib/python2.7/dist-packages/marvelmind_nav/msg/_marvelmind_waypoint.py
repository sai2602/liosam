# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from marvelmind_nav/marvelmind_waypoint.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class marvelmind_waypoint(genpy.Message):
  _md5sum = "c0d0bd68d8638aec15ccca2e2f6be8d8"
  _type = "marvelmind_nav/marvelmind_waypoint"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint8 total_items
uint8 item_index
uint8 movement_type
int16 param1
int16 param2
int16 param3
"""
  __slots__ = ['total_items','item_index','movement_type','param1','param2','param3']
  _slot_types = ['uint8','uint8','uint8','int16','int16','int16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       total_items,item_index,movement_type,param1,param2,param3

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(marvelmind_waypoint, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.total_items is None:
        self.total_items = 0
      if self.item_index is None:
        self.item_index = 0
      if self.movement_type is None:
        self.movement_type = 0
      if self.param1 is None:
        self.param1 = 0
      if self.param2 is None:
        self.param2 = 0
      if self.param3 is None:
        self.param3 = 0
    else:
      self.total_items = 0
      self.item_index = 0
      self.movement_type = 0
      self.param1 = 0
      self.param2 = 0
      self.param3 = 0

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
      _x = self
      buff.write(_get_struct_3B3h().pack(_x.total_items, _x.item_index, _x.movement_type, _x.param1, _x.param2, _x.param3))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 9
      (_x.total_items, _x.item_index, _x.movement_type, _x.param1, _x.param2, _x.param3,) = _get_struct_3B3h().unpack(str[start:end])
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
      _x = self
      buff.write(_get_struct_3B3h().pack(_x.total_items, _x.item_index, _x.movement_type, _x.param1, _x.param2, _x.param3))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 9
      (_x.total_items, _x.item_index, _x.movement_type, _x.param1, _x.param2, _x.param3,) = _get_struct_3B3h().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3B3h = None
def _get_struct_3B3h():
    global _struct_3B3h
    if _struct_3B3h is None:
        _struct_3B3h = struct.Struct("<3B3h")
    return _struct_3B3h
