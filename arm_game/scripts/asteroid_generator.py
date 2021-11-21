#!/usr/bin/env python3
#
#   asteroid_generator.py
#
#
import rospy

import rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import Point, Pose, Quaternion
import numpy as np


class AsteroidHandler:
    """
    AsteroidHandler handles the generation of asteroids in Gazebo and retrieving asteroid positions.
    For use, instantiate a AsteroidHandler object in the main code, and call functions as needed.
    """

    def __init__(self):
        self.num_asteroids_generated = 0  # Used for giving asteroids unique ID values
        self.asteroid_ID_list = []  # Stores the currently existing asteroid IDs

        # Set asteroid model path from package directory
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('arm_game')
        self.asteroid_model_path = package_path + "/meshes/RoboCup 3D Simulation Ball/model.sdf"

    def generate_asteroid(self, x, y, z):
        """
        Generate an asteroid in Gazebo at a given set of coordinates
        @param x: X coordinate of asteroid
        @param y: Y coordinate of asteroid
        @param z: Z coordinate of asteroid
        @return: spawn_model_client, int asteroid id
        """
        # Add an asteroid to the asteroid list with a unique ID
        self.num_asteroids_generated += 1
        self.asteroid_ID_list.append(self.num_asteroids_generated)

        try:
            # Spawn the asteroid in Gazebo at the specified point
            spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            return (spawn_model_client(
                model_name='asteroid' + str(self.num_asteroids_generated),
                model_xml=open(self.asteroid_model_path, 'r').read(),
                robot_namespace='/asteroids',
                initial_pose=Pose(position=Point(x, y, z), orientation=Quaternion(0, 0, 0, 0)),
                reference_frame='world'
            ), self.num_asteroids_generated)
        except rospy.ServiceException as e:
            print("Spawn Model service call failed: {0}".format(e))

    def generate_asteroid_gaussian(self, center_loc_x, center_loc_y, z, std_dev):
        """
        Generate an asteroid at a randomly-generated position based on a gaussian distribution
        @param center_loc_x: Defines the average x-location of the asteroid
        @param center_loc_y: Defines the average y-location of the asteroid
        @param z: Defines the starting height of the asteroid (Non-random)
        @param std_dev: Standard Deviation of gaussian
        @return: bool success, string status_message
        """
        x_loc = np.random.normal(center_loc_x, std_dev)
        y_loc = np.random.normal(center_loc_y, std_dev)
        return self.generate_asteroid(x_loc, y_loc, z)

    def generate_asteroid_random(self, center_loc_x, center_loc_y, z, max_dist):
        """
        Generate an asteroid at a randomly-generated position within a radius
        @param center_loc_x: Defines center x coordinate of asteroid generation
        @param center_loc_y: Defines center y coordinate of asteroid generation
        @param z: Defines the starting height of the asteroid (Non-random)
        @param max_dist: Defines the maximum distance from the center x,y that asteroid can generate at
        @return: bool success, string status_message
        """
        x_loc = (np.random.rand() - 0.5) * 2 * max_dist + center_loc_x
        y_loc = (np.random.rand() - 0.5) * 2 * max_dist + center_loc_y
        return self.generate_asteroid(x_loc, y_loc, z)

    def delete_asteroid(self, ID):
        """
        Deletes an existing asteroid from Gazebo
        @param ID: ID of asteroid to delete
        @return: bool success, string status_message
        """
        try:
            # Delete the asteroid
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

            return delete_model("asteroid" + str(ID))
        except rospy.ServiceException as e:
            print("Delete Model service call failed: {0}".format(e))

    def get_asteroid_state(self, ID):
        """
        Get the position and velocity components of a specific asteroid
        @param ID: ID of an asteroid to retrieve position for
        @return: State Object. To retrieve position and velocity components, use the following calls to the object:

        Position (X,Y,Z):
            state.pose.position.x, state.pose.position.y, state.pose.position.z
        Orientation (Quaternion):
            state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w
        Linear Velocity (X, Y, Z):
            state.twist.linear.x, state.twist.linear.y, state.twist.linear.z
        Angular Velocity (X, Y, Z):
            state.twist.angular.x, state.twist.angular.y, state.twist.angular.z
        """
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            # for block in self._blockListDict.itervalues():
            asteroid_name = "asteroid" + str(ID)
            resp_coordinates = model_coordinates(asteroid_name, "")  # Retrieve coordinates of asteroid in global frame
            return resp_coordinates

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

    def get_asteroid_position(self, ID):
        """
        Returns position in a nice form.
        """
        state = self.get_asteroid_state(ID)
        px = state.pose.position.x;
        py = state.pose.position.y;
        pz = state.pose.position.z;
        return np.array([px, py, pz]).reshape([3,1]);

    def get_asteroid_velocity(self, ID):
        """
        Returns velocity in a nice form.
        """
        state = self.get_asteroid_state(ID);
        vx = state.twist.linear.x;
        vy = state.twist.linear.y;
        vz = state.twist.linear.z;
        return np.array([vx, vy, vz]).reshape([3,1]);

class Asteroid:
    '''
    Asteroid is an interface class that holds a single asteroid and computes
    part of its trajectory. The class creates an asteroid at random and deletes
    it upon cleanup.

    Key functions:
    __init__()
    get_intercept_positions()
    '''

    '''
    Init: use an external handler and create a random asteroid from it.
    @param handler: AsteroidHandler object (shared between asteroids).
    @param workspace_radius: radius of workspace "sphere" approx. (float).
    @param t_start: time of asteroid creation (float). Does NOT determine when
                    asteroid created.
    '''
    def __init__(self, handler, workspace_radius, t_start):
        self.handler = handler
        # for now, just generate one at random. Save its ID.
        s, self.id = self.handler.generate_asteroid_random(0.0, 0.0, 5.0, 1.0)

        self.workspace_radius = workspace_radius

        # save key trajectory points
        self.t_start = t_start
        self.p0 = self.handler.get_asteroid_position(self.id);
        self.v0 = self.handler.get_asteroid_velocity(self.id);
        self.grav = np.array([0, 0, -9.8]).reshape([3,1]);      # TODO: FIX THIS TERM SO GRAVITY IS PULLED FROM GAZEBO

        # get intercept positions
        times = np.linspace(t_start, t_start + 10.0, num=1001)
        pos = self.get_position(times)
        posZ = pos[2]
        good_mask = np.where((posZ <  self.workspace_radius) & \
                            (posZ > -self.workspace_radius))
        good_times = times[good_mask]
        self.intercept_times = good_times

    '''
    Returns set of intercept times past t_current.
    '''
    def get_intercept_times(self, t_current):
        dt = t_current - self.t_start
        return self.intercept_times[self.intercept_times > dt]

    '''
    Calculates asteroid position at any time by extrapolating from constant
    acceleration kinematics.
    '''
    def get_position(self, t):
        dt = t - self.t_start;

        p = self.p0 + self.v0*dt + 0.5*self.grav*dt*dt;
        return p;

    '''
    Calculates asteroid velocity at any time by extrapolating from constant
    acceleration kinematics.
    '''
    def get_velocity(self, t):
        dt = t - self.t_start;

        v = self.v0 + self.grav*dt;
        return v;

    '''
    Deletes asteroid when object is destroyed.
    '''
    def remove(self):
        self.handler.delete_asteroid(self.id)


#
#  Main Code, used if script is run standalone
#  Contains a few testing functions
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('asteroid_handler')

    AH = AsteroidHandler()

    # Testing functions
    print(AH.generate_asteroid(1.0, 1.0, 1.0))
    print('\n')

    state = AH.get_asteroid_state(1)
    print("Asteroid state: " + str(state))
    print('\n')

    print(AH.delete_asteroid(1))
    print('\n')

    print(AH.generate_asteroid_random(0.0, 0.0, 5.0, 1.0))
    print(AH.generate_asteroid_gaussian(0.0, 0.0, 5.0, 1.0))

    print("-----")
    # using Asteroid:
    asteroid = Asteroid(AH, 1.0, 0.0)
    asteroid.remove()
