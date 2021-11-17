#!/usr/bin/env python3
#
#   asteroid_generator.py
#
#
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from gazebo_msgs.srv import SpawnModel, GetModelState
from geometry_msgs.msg import Point, Pose, Quaternion


class AsteroidHandler():
    """
    AsteroidHandler handles the generation of asteroids in Gazebo and retrieving asteroid positions.
    For use, instantiate a AsteroidHandler object in the main code, and call functions as needed.
    """
    numasteroidsgenerated = 0  # Used for giving asteroids unique ID values
    asteroid_ID_list = []  # Stores the currently existing asteroid IDs

    def generate_asteroid(self, x, y, z):
        """
        Generate an asteroid in Gazebo at a given set of coordinates
        :param x: X coordinate of asteroid
        :param y: Y coordinate of asteroid
        :param z: Z coordinate of asteroid
        :return: ID of last asteroid
        """
        # Add an asteroid to the asteroid list with a unique ID
        self.numasteroids += 1
        self.asteroid_ID_list.append(self.numasteroids)

        # Spawn the asteroid in Gazebo
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_client(
            model_name='asteroid' + str(self.numasteroids),
            model_xml=open('/home/robot/133ws/src/multipliciteam/arm_game/meshes/RoboCup 3D Simulation Ball/model.sdf', 'r').read(),
            robot_namespace='/foo',
            initial_pose=Pose(position=Point(x, y, z), orientation=Quaternion(0, 0, 0, 0)),
            reference_frame='world'
        )


    def remove_asteroid(self, ID)





    def get_asteroid_position(self, ID):
        """
        Get the position coordinates of a specific asteroid
        :param ID: ID of an asteroid to retrieve position for
        :return: Position class. Retrieve coordinates via: Position.x, Position.y, Position.z
        """

        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            # for block in self._blockListDict.itervalues():
            asteroidName = "asteroid" + str(ID)
            resp_coordinates = model_coordinates(asteroidName, "")  # Retrieve coordinates of asteroid in global frame
            print
            '\n'
            print
            'Status.success = ', resp_coordinates.success
            print(asteroidName)
            print("Position X: " + str(resp_coordinates.pose.position.x))
            print("Position Y: " + str(resp_coordinates.pose.position.y))
            print("Position Z: " + str(resp_coordinates.pose.position.z))
            print("Orientation : " + str(resp_coordinates.pose.orientation.x))

            return resp_coordinates.pose.position
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))




#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('asteroid_handler')

    AH = AsteroidHandler()

    # testing
    AH.generate_asteroid(1.0,1.0,1.0)
    AH.get_asteroid_position(1)

    # Run until shutdown (killed or ctrl-C'ed).  Note the relay action
    # (from GUI to Gazebo) is triggered when a new message arrives
    # from the GUI.  Hence we do not need a servo loop and can just
    # "spin" here (which allows sepereate threads to listen for
    # incoming messages).
    rospy.loginfo("Running...")
    rospy.spin()
