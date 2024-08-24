import math
import copy

import rospy
import rosgraph
import hardware_integration.util as util

from sensor_msgs.msg import NavSatFix, Imu, Range, LaserScan
from geographic_msgs.msg import GeoPoseStamped, GeoPoint
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, ParamSet
from pygeodesy.geoids import GeoidPGM
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64


class HardwareInterface:
    def __init__(self) -> None:
        if not rosgraph.is_master_online():
            raise ConnectionError("ROS MASTER is not online, did you start roscore/simulation?")

        # Interface State
        self.connection_initialized = False
        self.internal_state = "uninitialized"
        self._flight_mode = util.get_flight_mode()

        # Drone State
        self.current_state = State()      # Current Flight State of UAV
        self.abs_pose = NavSatFix()       # Absolute Current Position SAT Coordinates
        self.cur_pose = GeoPoseStamped()  # Current Position Coordinates
        self.start_pose = GeoPoseStamped  # Position when connection was initialized (might not be "actual" drone start)
        self.compass_heading = Float64()  # Compass Heading
        self.range_sensor_h = 0           # Distance Detected by Range Sensor
        self.imu_orientation = Imu()      # IMU Based orientation of drone

        self._state_sub = rospy.Subscriber("mavros/state", State, callback=self._state_cb)
        self._global_pos_sub = rospy.Subscriber("mavros/global_position/global", NavSatFix, callback=self._abs_pose_cb)
        self._compass_sub = rospy.Subscriber("mavros/global_position/compass_hdg", Float64, callback=self._compass_heading_cb)
        if not self._flight_mode.is_simulation:
            self._range_sub = rospy.Subscriber('/teraranger_evo', Range, callback=self._range_sensor_cb)
        else:
            self._range_sub = rospy.Subscriber('/laser_1/scan', LaserScan, callback=self._range_sensor_cb)
        self._imu_sub = rospy.Subscriber("mavros/imu/data", Imu, callback=self._imu_cb)

        self._global_pos_pub = rospy.Publisher("mavros/setpoint_position/global", GeoPoseStamped, queue_size=10)

        # TODO: rate.sleeping in here will causes delays (might impact something else idk)
        self._rate = rospy.Rate(20)

        # Done Parameters
        # TODO: these must be adjustable by a param file or something for real flight parameters
        self.camera_angle = 0           # Angle of Camera Relative to Heading CCW
        self.start_cruise_velocity = 2  # Cruise Velocity in m/s

        # Parameters for control
        self.target_pose = GeoPoseStamped()  # Global position for drone to navigate to
        # TODO: coord tolerance adjustable by param (also through method?)
        # self.coord_tolerance = 3.6e-5  # Coordinate Tolerance, ~4m
        self.coord_tolerance = 3.6e-6  # Coordinate Tolerance, ~20cm
        # self.alt_tolerance = 0.3  # Altitude Tolerance
        self.alt_tolerance = 0.2  # Altitude Tolerance

        self.lat_factor = 111320        # Constant Number of Meters per Degree Latitude
        self.long_factor = 40075 * 1000 * math.cos(math.radians(self.cur_pose.pose.position.latitude)) / 360  # Non-constant Number of Meters per Degree Longitude

        print("Interface created")

    def initialize_connection(self) -> None:
        print("Initializing connection")
        rospy.wait_for_service("/mavros/cmd/arming")
        self._arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self._set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        rospy.wait_for_service("/mavros/cmd/land")
        self._landing_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        print("Services created")

        # Init offboard mode
        # https://docs.px4.io/main/en/ros/mavros_offboard_python.html
        print("Waiting for controller")
        while (not rospy.is_shutdown() and not self.current_state.connected):
            self._rate.sleep()

        self._offb_set_mode = SetModeRequest()
        self._offb_set_mode.custom_mode = 'OFFBOARD'
        self._arm_cmd = CommandBoolRequest()
        self._arm_cmd.value = True
        self._last_req = rospy.Time.now()

        self._wait_for_offboard_and_arm()

        self.set_horizontal_velocity(self.start_cruise_velocity)  # Cruise Velocity
        self._set_start_pose()
        self.target_pose = copy.deepcopy(self.start_pose)

        self._set_internal_state("initialized")
        print("Interface initialized")

    def _wait_for_offboard_and_arm(self):
        print("Waiting for arm")
        while not self._arm_vehicle():
            self._rate.sleep()
            continue

        print("Initializing offboard")
        geo_pose = GeoPoseStamped()
        for i in range(100):
            if (rospy.is_shutdown()):
                break
            self._global_pos_pub.publish(geo_pose)
            self._rate.sleep()

        print("Waiting for offboard")
        while not self._set_offboard_mode():
            self._rate.sleep()
            continue

    def cycle(self) -> None:
        self._publish_target()

    def _set_offboard_mode(self) -> bool:
        if (self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - self._last_req) > rospy.Duration(5.0)):
            if (self._set_mode_client.call(self._offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
                return True
            self._last_req = rospy.Time.now()
        return False

    def _arm_vehicle(self) -> bool:
        if (not self.current_state.armed):
            if (self._arming_client.call(self._arm_cmd).success == True):
                rospy.loginfo("Vehicle armed")
                return True
        return False

    def _set_internal_state(self, state: str) -> None:
        self.internal_state = state

    def set_target_pose(self, msg: GeoPoseStamped) -> None:
        self.set_target_position(msg.pose.position)
        # mavros only extracts yaw
        self.set_target_orientation(msg.pose.orientation)

    def set_target_position(self, msg: GeoPoint) -> None:
        self.target_pose.pose.position.latitude = msg.latitude
        self.target_pose.pose.position.longitude = msg.longitude
        self.target_pose.pose.position.altitude = msg.altitude

    def set_target_orientation(self, msg: Quaternion) -> None:
        # mavros only extracts yaw
        self.target_pose.pose.orientation.x = msg.x
        self.target_pose.pose.orientation.y = msg.y
        self.target_pose.pose.orientation.z = msg.z
        self.target_pose.pose.orientation.w = msg.w

    def set_target_position_loc(self, dx=0, dy=0, dz=0):
        self.target_pose.pose.position.latitude += dx / self.lat_factor
        self.target_pose.pose.position.longitude += dy / self.long_factor
        self.target_pose.pose.position.altitude += dz

    def set_target_position_rel(self, msg: GeoPoint, dx=0, dy=0, dz=0) -> None:
        msg = copy.deepcopy(msg)
        msg.latitude += dx / self.lat_factor
        msg.longitude += dy / self.long_factor
        msg.altitude += dz
        self.set_target_position(msg)

    def _publish_target(self) -> None:
        self._set_internal_state("waypoint")
        self._global_pos_pub.publish(self.target_pose)

    # TODO
    def continuous_flight(self):
        pass

    # TODO
    def set_yaw(self):
        pass

    def set_horizontal_velocity(self, max_velocity) -> None:
        rospy.wait_for_service('/mavros/param/set')
        try:
            max_hor_vel = rospy.ServiceProxy('/mavros/param/set', ParamSet)
            if self._flight_mode.is_real:
                max_hor_vel(param_id="MPC_XY_VEL_CRUISE", value=ParamValue(real=max_velocity))
            else:
                max_hor_vel(param_id="MPC_XY_VEL_ALL", value=ParamValue(real=max_velocity))
        except rospy.ServiceException as e:
            print("Service max_horizontal_velocity (MPC_XY_VEL) call failed: %s" % e)

    def has_arrived_target(self, epsilon_coord=None, epsilon_alt=None) -> bool:
        return self.has_arrived_position(self.target_pose.pose.position)

    def has_arrived_position(self, pose: GeoPoint, epsilon_coord=None, epsilon_alt=None) -> bool:
        epsilon_coord = epsilon_coord or self.coord_tolerance
        epsilon_alt = epsilon_alt or self.alt_tolerance

        if abs(self.cur_pose.pose.position.latitude - pose.latitude) <= epsilon_coord and abs(self.cur_pose.pose.position.longitude - pose.longitude) <= epsilon_coord \
                and abs(self.cur_pose.pose.position.altitude - pose.altitude) <= epsilon_alt:
            return True
        else:
            return False

    # TODO:
    def has_arrived_orientation(self) -> bool:
        pass

    def land_drone(self) -> None:
        self._set_internal_state("landing")
        response = self._landing_client(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
        rospy.loginfo(response)

    # CALLBACKS

    # Get Altitude Above Sea Level Difference
    # https://wiki.ros.org/mavros#mavros.2FPlugins.Avoiding_Pitfalls_Related_to_Ellipsoid_Height_and_Height_Above_Mean_Sea_Level
    _egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

    def _geoid_height(self, lat, lon) -> float:
        """Calculates AMSL to ellipsoid conversion offset.
        Uses EGM96 data with 5' grid and cubic interpolation.
        The value returned can help you convert from meters
        above mean sea level (AMSL) to meters above
        the WGS84 ellipsoid.

        If you want to go from AMSL to ellipsoid height, add the value.

        To go from ellipsoid height to AMSL, subtract this value.
        """
        return self._egm96.height(lat, lon)

    def _state_cb(self, msg: State) -> None:
        self.current_state = msg

    def _abs_pose_cb(self, msg: NavSatFix) -> None:
        self.abs_pose = msg
        self.cur_pose.pose.position.latitude = self.abs_pose.latitude
        self.cur_pose.pose.position.longitude = self.abs_pose.longitude
        self.cur_pose.pose.position.altitude = self.abs_pose.altitude - self._geoid_height(self.abs_pose.latitude, self.abs_pose.longitude)
        self.lat_factor = 111320
        self.long_factor = 40075 * 1000 * math.cos(math.radians(self.cur_pose.pose.position.latitude)) / 360

    def _set_start_pose(self) -> None:
        self.start_pose = copy.deepcopy(self.cur_pose)
        self.start_pose.pose.orientation = copy.deepcopy(self.imu_orientation.orientation)

    def _compass_heading_cb(self, msg: Float64) -> None:
        self.compass_heading = msg

    def _range_sensor_cb(self, msg) -> None:
        if self._flight_mode.is_real:
            self.range_sensor_h = msg.range
        else:
            self.range_sensor_h = msg.ranges[0]

    def _imu_cb(self, msg: Imu) -> None:
        self.imu_orientation = msg
        self.cur_pose.pose.orientation.x = msg.orientation.x
        self.cur_pose.pose.orientation.y = msg.orientation.y
        self.cur_pose.pose.orientation.z = msg.orientation.z
        self.cur_pose.pose.orientation.w = msg.orientation.w

    # MISC (subject to change)

    def get_flight_info(self) -> str:
        endchar = "\n"
        message = (
            "STARTING POSITION:" + endchar +
            str(self.start_pose.pose.position.latitude) + endchar +
            str(self.start_pose.pose.position.longitude) + endchar +
            str(self.start_pose.pose.position.altitude) + endchar +
            "TARGET WAYPOINT:" + endchar +
            "------------------------------------------------------" + endchar +
            str(self.target_pose.pose.position.latitude) + endchar +
            str(self.target_pose.pose.position.longitude) + endchar +
            str(self.target_pose.pose.position.altitude) + endchar +
            "------------------------------------------------------" + endchar +
            "TARGET DIFF TO CUR:" + endchar +
            str((self.target_pose.pose.position.latitude - self.cur_pose.pose.position.latitude) * self.lat_factor) + endchar +
            str((self.target_pose.pose.position.longitude - self.cur_pose.pose.position.longitude) * self.long_factor) + endchar +
            str((self.target_pose.pose.position.altitude - self.cur_pose.pose.position.altitude)) + endchar +
            "------------------------------------------------------" + endchar +
            "CURRENT POSITION:" + endchar +
            str(self.cur_pose.pose.position.latitude) + endchar +
            str(self.cur_pose.pose.position.longitude) + endchar +
            str(self.cur_pose.pose.position.altitude) + endchar +
            "------------------------------------------------------" + endchar +
            "CURRENT DIFF TO START:" + endchar +
            str((self.cur_pose.pose.position.latitude - self.start_pose.pose.position.latitude) * self.lat_factor) + endchar +
            str((self.cur_pose.pose.position.longitude - self.start_pose.pose.position.longitude) * self.long_factor) + endchar +
            str((self.cur_pose.pose.position.altitude - self.start_pose.pose.position.altitude)) + endchar +
            "------------------------------------------------------" + endchar +
            "MODE: " + str(self.current_state.mode) + endchar +
            "ARMED: " + str(self.current_state.armed) + endchar +
            "INTERNAL STATE: " + str(self.internal_state) + endchar +
            "HAS ARRIVED: " + str(self.has_arrived_position(self.target_pose.pose.position)) + endchar +
            "HEADING: " + str(self.compass_heading.data) + endchar +
            "CAMERA ANGLE: " + str(self.camera_angle) + endchar +
            "RANGE DISTANCE: " + str(self.range_sensor_h) + endchar +
            "------------------------------------------------------" + endchar
        )
        return message
