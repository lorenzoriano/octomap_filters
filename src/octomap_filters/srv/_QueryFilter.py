"""autogenerated by genpy from octomap_filters/QueryFilterRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class QueryFilterRequest(genpy.Message):
  _md5sum = "c1f3d28f1b044c871e6eff2e9fc3c667"
  _type = "octomap_filters/QueryFilterRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
string name


"""
  __slots__ = ['name']
  _slot_types = ['string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       name

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(QueryFilterRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.name is None:
        self.name = ''
    else:
      self.name = ''

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
      _x = self.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
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
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name = str[start:end].decode('utf-8')
      else:
        self.name = str[start:end]
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
      _x = self.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
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
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name = str[start:end].decode('utf-8')
      else:
        self.name = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
"""autogenerated by genpy from octomap_filters/QueryFilterResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class QueryFilterResponse(genpy.Message):
  _md5sum = "93aa3d73b866f04880927745f4aab303"
  _type = "octomap_filters/QueryFilterResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
geometry_msgs/Point min


geometry_msgs/Point max



================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

"""
  __slots__ = ['min','max']
  _slot_types = ['geometry_msgs/Point','geometry_msgs/Point']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       min,max

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(QueryFilterResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.min is None:
        self.min = geometry_msgs.msg.Point()
      if self.max is None:
        self.max = geometry_msgs.msg.Point()
    else:
      self.min = geometry_msgs.msg.Point()
      self.max = geometry_msgs.msg.Point()

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
      buff.write(_struct_6d.pack(_x.min.x, _x.min.y, _x.min.z, _x.max.x, _x.max.y, _x.max.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.min is None:
        self.min = geometry_msgs.msg.Point()
      if self.max is None:
        self.max = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 48
      (_x.min.x, _x.min.y, _x.min.z, _x.max.x, _x.max.y, _x.max.z,) = _struct_6d.unpack(str[start:end])
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
      _x = self
      buff.write(_struct_6d.pack(_x.min.x, _x.min.y, _x.min.z, _x.max.x, _x.max.y, _x.max.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.min is None:
        self.min = geometry_msgs.msg.Point()
      if self.max is None:
        self.max = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 48
      (_x.min.x, _x.min.y, _x.min.z, _x.max.x, _x.max.y, _x.max.z,) = _struct_6d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_6d = struct.Struct("<6d")
class QueryFilter(object):
  _type          = 'octomap_filters/QueryFilter'
  _md5sum = '4b8e0b34111eb0c24b091f6caefacb3c'
  _request_class  = QueryFilterRequest
  _response_class = QueryFilterResponse
