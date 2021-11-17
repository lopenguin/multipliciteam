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
        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path('arm_game')
        self.asteroid_model_path = self.package_path + "/meshes/RoboCup 3D Simulation Ball/model.sdf"

    def generate_asteroid(self, x, y, z):
        """
        Generate an asteroid in Gazebo at a given set of coordinates
        @param x: X coordinate of asteroid
        @param y: Y coordinate of asteroid
        @param z: Z coordinate of asteroid
        @return: bool success, string status_message
        """
        # Add an asteroid to the asteroid list with a unique ID
        self.num_asteroids_generated += 1
        self.asteroid_ID_list.append(self.num_asteroids_generated)

        try:
            # Spawn the asteroid in Gazebo at the specified point
            spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            return spawn_model_client(
                model_name='asteroid' + str(self.num_asteroids_generated),
                model_xml=open(self.asteroid_model_path, 'r').read(),
                robot_namespace='/asteroids',
                initial_pose=Pose(position=Point(x, y, z), orientation=Quaternion(0, 0, 0, 0)),
                reference_frame='world'
            )
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

    def get_asteroid_position(self, ID):
        """
        Get the position coordinates of a specific asteroid
        @param ID: ID of an asteroid to retrieve position for
        @return: Position class. Retrieve coordinates via: Position.x, Position.y, Position.z
        """
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            # for block in self._blockListDict.itervalues():
            asteroid_name = "asteroid" + str(ID)
            resp_coordinates = model_coordinates(asteroid_name, "")  # Retrieve coordinates of asteroid in global frame

            return resp_coordinates.pose.position
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))


#
#  Main Code, used if function is run standalone
#  Contains a few testing functions
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('asteroid_handler')

    AH = AsteroidHandler()

    # Testing functions
    print(AH.generate_asteroid(1.0, 1.0, 1.0))
    print('\n')

    pos = AH.get_asteroid_position(1)
    print("Position X: " + str(pos.x))
    print("Position Y: " + str(pos.y))
    print("Position Z: " + str(pos.z))
    print('\n')

    print(AH.delete_asteroid(1))
    print('\n')

    print(AH.generate_asteroid_random(0.0, 0.0, 5.0, 1.0))
    print(AH.generate_asteroid_gaussian(0.0, 0.0, 5.0, 1.0))


