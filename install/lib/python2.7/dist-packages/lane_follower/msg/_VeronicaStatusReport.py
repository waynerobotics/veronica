# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from lane_follower/VeronicaStatusReport.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class VeronicaStatusReport(genpy.Message):
  _md5sum = "91482b64151ad166efd0cf38c80d57a6"
  _type = "lane_follower/VeronicaStatusReport"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool IMU_STATUS
bool CAMERA_STATUS
bool GPS_STATUS
bool LIDAR_STATUS
bool MOTOR_DRIVER_STATUS

"""
  __slots__ = ['IMU_STATUS','CAMERA_STATUS','GPS_STATUS','LIDAR_STATUS','MOTOR_DRIVER_STATUS']
  _slot_types = ['bool','bool','bool','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       IMU_STATUS,CAMERA_STATUS,GPS_STATUS,LIDAR_STATUS,MOTOR_DRIVER_STATUS

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(VeronicaStatusReport, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.IMU_STATUS is None:
        self.IMU_STATUS = False
      if self.CAMERA_STATUS is None:
        self.CAMERA_STATUS = False
      if self.GPS_STATUS is None:
        self.GPS_STATUS = False
      if self.LIDAR_STATUS is None:
        self.LIDAR_STATUS = False
      if self.MOTOR_DRIVER_STATUS is None:
        self.MOTOR_DRIVER_STATUS = False
    else:
      self.IMU_STATUS = False
      self.CAMERA_STATUS = False
      self.GPS_STATUS = False
      self.LIDAR_STATUS = False
      self.MOTOR_DRIVER_STATUS = False

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
      buff.write(_get_struct_5B().pack(_x.IMU_STATUS, _x.CAMERA_STATUS, _x.GPS_STATUS, _x.LIDAR_STATUS, _x.MOTOR_DRIVER_STATUS))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 5
      (_x.IMU_STATUS, _x.CAMERA_STATUS, _x.GPS_STATUS, _x.LIDAR_STATUS, _x.MOTOR_DRIVER_STATUS,) = _get_struct_5B().unpack(str[start:end])
      self.IMU_STATUS = bool(self.IMU_STATUS)
      self.CAMERA_STATUS = bool(self.CAMERA_STATUS)
      self.GPS_STATUS = bool(self.GPS_STATUS)
      self.LIDAR_STATUS = bool(self.LIDAR_STATUS)
      self.MOTOR_DRIVER_STATUS = bool(self.MOTOR_DRIVER_STATUS)
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
      buff.write(_get_struct_5B().pack(_x.IMU_STATUS, _x.CAMERA_STATUS, _x.GPS_STATUS, _x.LIDAR_STATUS, _x.MOTOR_DRIVER_STATUS))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 5
      (_x.IMU_STATUS, _x.CAMERA_STATUS, _x.GPS_STATUS, _x.LIDAR_STATUS, _x.MOTOR_DRIVER_STATUS,) = _get_struct_5B().unpack(str[start:end])
      self.IMU_STATUS = bool(self.IMU_STATUS)
      self.CAMERA_STATUS = bool(self.CAMERA_STATUS)
      self.GPS_STATUS = bool(self.GPS_STATUS)
      self.LIDAR_STATUS = bool(self.LIDAR_STATUS)
      self.MOTOR_DRIVER_STATUS = bool(self.MOTOR_DRIVER_STATUS)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_5B = None
def _get_struct_5B():
    global _struct_5B
    if _struct_5B is None:
        _struct_5B = struct.Struct("<5B")
    return _struct_5B
