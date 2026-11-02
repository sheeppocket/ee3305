"""Small message/executor stand-ins for algorithm tests, not a ROS emulator."""
import importlib
import sys
from copy import deepcopy
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch


class Stamp:
    def __init__(self, sec=1, nanosec=0):
        self.sec, self.nanosec = sec, nanosec

    def __eq__(self, other):
        return isinstance(other, Stamp) and (self.sec, self.nanosec) == (other.sec, other.nanosec)


def header():
    return NS(frame_id='', stamp=Stamp())


def pose():
    return NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))


class PoseStamped:
    def __init__(self):
        self.header, self.pose = header(), pose()


class Path:
    def __init__(self):
        self.header, self.poses = header(), []


class Odometry:
    def __init__(self):
        self.header = header()
        self.pose = NS(pose=pose())


class OccupancyGrid:
    def __init__(self):
        self.header = header()
        self.info = NS(width=0, height=0, resolution=.1, origin=pose())
        self.data = []


class TwistStamped:
    def __init__(self):
        self.header = header()
        self.twist = NS(linear=NS(x=0., y=0., z=0.), angular=NS(x=0., y=0., z=0.))


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(deepcopy(msg))


class Clock:
    def now(self):
        return NS(to_msg=lambda: Stamp(10), nanoseconds=10_000_000_000)


class Node:
    def __init__(self, name):
        self.parameters = {}

    def declare_parameter(self, name, value):
        self.parameters[name] = value

    def get_parameter(self, name):
        return NS(value=self.parameters[name])

    def create_publisher(self, *args):
        return Publisher()

    def create_subscription(self, *args):
        return None

    def create_timer(self, *args):
        return None

    def get_clock(self):
        return Clock()

    def get_logger(self):
        return NS(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None)


class Time:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds

    @classmethod
    def from_msg(cls, stamp):
        return cls(stamp.sec*10**9 + stamp.nanosec)

    def __sub__(self, other):
        return Time(self.nanoseconds-other.nanoseconds)


class FakeClock(Clock):
    def now(self):
        value = Time(10_000_000_000)
        value.to_msg = lambda: Stamp(10)
        return value


Node.get_clock = lambda self: FakeClock()


def load_modules():
    names = ['rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.time', 'geometry_msgs',
             'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg', 'sensor_msgs',
             'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg', 'tf2_ros']
    modules = {name: ModuleType(name) for name in names}
    modules['rclpy.node'].Node = Node
    qos = modules['rclpy.qos']
    qos.QoSProfile = lambda **kwargs: NS(**kwargs)
    qos.DurabilityPolicy = NS(TRANSIENT_LOCAL=1)
    qos.qos_profile_sensor_data = NS()
    qos.qos_profile_services_default = NS(history=1, depth=10, reliability=1)
    modules['rclpy.time'].Time = Time
    for name, attrs in [('geometry_msgs.msg', [PoseStamped, TwistStamped]),
                        ('nav_msgs.msg', [OccupancyGrid, Odometry, Path])]:
        for cls in attrs:
            setattr(modules[name], cls.__name__, cls)
    modules['sensor_msgs.msg'].LaserScan = object
    modules['std_msgs.msg'].Bool = lambda data=False: NS(data=data)
    modules['std_msgs.msg'].String = lambda data='': NS(data=data)
    modules['tf2_ros'].Buffer = lambda: NS()
    modules['tf2_ros'].TransformListener = lambda *args: None
    modules['tf2_ros'].TransformException = RuntimeError
    with patch.dict(sys.modules, modules):
        slam = importlib.import_module('ee3305_py.online_slam')
        safety = importlib.import_module('ee3305_py.online_safety')
        adapters = importlib.import_module('ee3305_py.online_adapters')
        return [NS(Planner=adapters.OnlinePlanner),
                NS(Controller=adapters.OnlineController), slam, safety]


def map_message(grid):
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.header.stamp = Stamp(10)
    msg.info.width, msg.info.height = grid.width, grid.height
    msg.info.resolution = grid.resolution
    msg.info.origin.position.x, msg.info.origin.position.y = grid.origin_x, grid.origin_y
    msg.data = list(grid.data)
    return msg


def request(start, goal):
    msg = Path()
    msg.header.frame_id = 'map'
    for x, y in [start, goal]:
        p = PoseStamped()
        p.pose.position.x, p.pose.position.y = x, y
        msg.poses.append(p)
    return msg
