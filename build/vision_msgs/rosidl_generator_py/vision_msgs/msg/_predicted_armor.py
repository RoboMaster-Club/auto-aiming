# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vision_msgs:msg/PredictedArmor.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PredictedArmor(type):
    """Metaclass of message 'PredictedArmor'."""

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
                'vision_msgs.msg.PredictedArmor')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__predicted_armor
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__predicted_armor
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__predicted_armor
            cls._TYPE_SUPPORT = module.type_support_msg__msg__predicted_armor
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__predicted_armor

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


class PredictedArmor(metaclass=Metaclass_PredictedArmor):
    """Message class 'PredictedArmor'."""

    __slots__ = [
        '_header',
        '_x',
        '_y',
        '_z',
        '_pitch',
        '_yaw',
        '_roll',
        '_x_vel',
        '_y_vel',
        '_z_vel',
        '_fire',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'x': 'double',
        'y': 'double',
        'z': 'double',
        'pitch': 'double',
        'yaw': 'double',
        'roll': 'double',
        'x_vel': 'double',
        'y_vel': 'double',
        'z_vel': 'double',
        'fire': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.z = kwargs.get('z', float())
        self.pitch = kwargs.get('pitch', float())
        self.yaw = kwargs.get('yaw', float())
        self.roll = kwargs.get('roll', float())
        self.x_vel = kwargs.get('x_vel', float())
        self.y_vel = kwargs.get('y_vel', float())
        self.z_vel = kwargs.get('z_vel', float())
        self.fire = kwargs.get('fire', bool())

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
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        if self.pitch != other.pitch:
            return False
        if self.yaw != other.yaw:
            return False
        if self.roll != other.roll:
            return False
        if self.x_vel != other.x_vel:
            return False
        if self.y_vel != other.y_vel:
            return False
        if self.z_vel != other.z_vel:
            return False
        if self.fire != other.fire:
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
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
        self._x = value

    @property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
        self._y = value

    @property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z' field must be of type 'float'"
        self._z = value

    @property
    def pitch(self):
        """Message field 'pitch'."""
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch' field must be of type 'float'"
        self._pitch = value

    @property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw' field must be of type 'float'"
        self._yaw = value

    @property
    def roll(self):
        """Message field 'roll'."""
        return self._roll

    @roll.setter
    def roll(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll' field must be of type 'float'"
        self._roll = value

    @property
    def x_vel(self):
        """Message field 'x_vel'."""
        return self._x_vel

    @x_vel.setter
    def x_vel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_vel' field must be of type 'float'"
        self._x_vel = value

    @property
    def y_vel(self):
        """Message field 'y_vel'."""
        return self._y_vel

    @y_vel.setter
    def y_vel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_vel' field must be of type 'float'"
        self._y_vel = value

    @property
    def z_vel(self):
        """Message field 'z_vel'."""
        return self._z_vel

    @z_vel.setter
    def z_vel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_vel' field must be of type 'float'"
        self._z_vel = value

    @property
    def fire(self):
        """Message field 'fire'."""
        return self._fire

    @fire.setter
    def fire(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fire' field must be of type 'bool'"
        self._fire = value
