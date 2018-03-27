import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


def read_grid_anchor_global_pos(filename):
    '''
    Read and parse the first line of the given file which must have the following format:
    'lat0 <lat_in_decimal_degrees>, lon0 <lon_in_decimal_degrees>
    and return a numpy array containing the longitude, latitude and 0 (altitude).
    :param filename: input filename
    :return: a numpy array containing the longitude, latitude and 0 (altitude).
    '''
    with open(filename) as input:
        line = next(input)
        _, lat_str, _, lon_str = line.split(" ")
        return np.array([float(lon_str), float(lat_str.strip(',')), 0])


def local_to_grid(local_pos, north_offset, east_offset):
    """
    Convert a local position in the current NED frame (center in the home position)
    into the grid index of the cell containing that position.
    :param local_pos: a position in the current local NED frame
    :return: the grid coordinates of the cell containing that position.
    """
    return int(np.floor(local_pos[0])) - north_offset, int(np.floor(local_pos[1])) - east_offset


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        # Read the obstacle map anchor position and set as home position.
        # Setting the home position fix the local NED frame
        grid_anchor_global_pos = read_grid_anchor_global_pos('colliders.csv')
        self.set_home_position(*grid_anchor_global_pos)
        print('home global position  {0}'.format(self.global_home))

        # Read the obstacle data and create a config space grid
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Convert the current global position of the drone into a (north,east) offset from the grid center
        drone_local_pos = global_to_local(self.global_position, self.global_home)
        drone_grid_pos = local_to_grid(drone_local_pos, north_offset, east_offset)
        print('drone global position {0}\n'
              'drone local position  {1}\n'
              'drone grid position   {2}'.format(self.global_position, drone_local_pos, drone_grid_pos))

        # Set arbitrary goal in global coordinates
        # and convert to a grid cell goal
        goal_global_pos = np.array([-122.400150, 37.796005, 0])
        goal_local_pos = global_to_local(goal_global_pos, self.global_home)
        goal_grid_pos = local_to_grid(goal_local_pos, north_offset, east_offset)
        print('goal global position {0}\n'
              'goal local position  {1}\n'
              'goal grid position   {2}'.format(goal_global_pos, goal_local_pos, goal_grid_pos))

        # Run A* to find a path from start to goal
        # or move to a different search space such as a graph (not done here)
        print('Grid Start and Goal: ', drone_grid_pos, goal_grid_pos)
        path, _ = a_star(grid, heuristic, drone_grid_pos, goal_grid_pos)
        print('Path length before pruning = {}'.format(len(path)))
        path = prune_path(path, grid)
        print('Path length after pruning = {}'.format(len(path)))

        # set the target altitude
        self.target_position[2] = TARGET_ALTITUDE

        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints in the local NED frame
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        self.waypoints = waypoints
        # send waypoints to sim to visualize the path
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
