# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vision_msgs:msg/KeyPoints.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'points'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_KeyPoints(type):
    """Metaclass of message 'KeyPoints'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('vision_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vision_msgs.msg.KeyPoints')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__key_points
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__key_points
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__key_points
            cls._TYPE_SUPPORT = module.type_support_msg__msg__key_points
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__key_points

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class KeyPoints(metaclass=Metaclass_KeyPoints):
    """Message class 'KeyPoints'."""

    __slots__ = [
        '_header',
        '_points',
        '_is_large_armor',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'points': 'float[8]',
        'is_large_armor': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 8),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        if 'points' not in kwargs:
            self.points = numpy.zeros(8, dtype=numpy.float32)
        else:
            self.points = numpy.array(kwargs.get('points'), dtype=numpy.float32)
            assert self.points.shape == (8, )
        self.is_large_armor = kwargs.get('is_large_armor', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if all(self.points != other.points):
            return False
        if self.is_large_armor != other.is_large_armor:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @property
    def points(self):
        """Message field 'points'."""
        return self._points

    @points.setter
    def points(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'points' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 8, \
                "The 'points' numpy.ndarray() must have a size of 8"
            self._points = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 8 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'points' field must be a set or sequence with length 8 and each value of type 'float'"
        self._points = numpy.array(value, dtype=numpy.float32)

    @property
    def is_large_armor(self):
        """Message field 'is_large_armor'."""
        return self._is_large_armor

    @is_large_armor.setter
    def is_large_armor(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_large_armor' field must be of type 'bool'"
        self._is_large_armor = value
