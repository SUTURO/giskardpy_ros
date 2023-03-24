import rospy
from geometry_msgs.msg import PoseStamped, Pose
from giskard_msgs.srv import UpdateWorldRequest, UpdateWorld
from py_trees import Status

from giskardpy.model.utils import make_world_body_box, make_world_body_cylinder, make_world_body_sphere
from giskardpy.tree.behaviors.plugin import GiskardBehavior

from rosprolog_client import Prolog

#import pydevd_pycharm


# pydevd_pycharm.settrace('localhost', port=1234, stdoutToServer=True, stderrToServer=True, suspend=False)
from giskardpy.utils import logging


class SuturoWorldSynchroniser(GiskardBehavior):
    prolog: Prolog
    last_update: float

    @profile
    def __init__(self, name=None,
                 name_space='rosprolog'):
        if name is None:
            name = 'suturo knowledge world poller'
        super().__init__(name)
        self.name_space = name_space

    @profile
    def setup(self, timeout=0.0):
        self.prolog = Prolog(name_space=self.name_space,
                             wait_for_services=False)
        self.last_update = 0.0
        return True

    # shape terms:
    # mesh(File, [X,Y,Z])
    # mesh with filename and scale. Won't be used in milestone 2
    #
    # box(X,Y,Z)
    # box with depth, width, and height. Will be used in milestone 2
    #
    # cylinder(Radius,Length)
    #
    # sphere(Radius)

    def poll(self):
        updates = self.prolog.once(f'giskard_updates({self.last_update}, CurrentTime, Updates)')
        self.last_update = updates['CurrentTime']
        # print(updates, flush=True)
        # logging.loginfo(updates)

        for element in updates['Updates']:
            try:
                if isinstance(element[3], str):
                    print('Bugged element with string instead of dict')
                    continue
                else:
                    obj = element[3]['term']

                obj_name = str(element[2][0])

                # Create object body
                if obj[0] == 'box':
                    obj_body = make_world_body_box(obj[1], obj[2], obj[3])
                    obj_msg = 'added box'
                elif obj[0] == 'cylinder':
                    obj_body = make_world_body_cylinder(obj[2], obj[1])
                    obj_msg = 'added cylinder'
                elif obj[0] == 'sphere':
                    obj_body = make_world_body_sphere(obj[1])
                    obj_msg = 'added sphere'
                else:
                    logging.loginfo('No object body to add')
                    continue

                logging.loginfo(obj_msg)

                # Object pose
                obj_pose = Pose()
                obj_pose.position.x = element[2][1][0]
                obj_pose.position.y = element[2][1][1]
                obj_pose.position.z = element[2][1][2]
                obj_pose.orientation.x = element[2][2][0]
                obj_pose.orientation.y = element[2][2][1]
                obj_pose.orientation.z = element[2][2][2]
                obj_pose.orientation.w = element[2][2][3]

                parent_link = self.world.root_link_name
                # Parent link
                obj_parent_link_group = ''
                obj_parent_link = ''
                #parent_link = self.world.get_link_name(obj_parent_link, obj_parent_link_group)

                self.world.add_world_body(obj_name, obj_body, obj_pose, parent_link)

                # code to update self collision matrix when objects are attached to the robot
                # parent_group = self.world.get_parent_group_name(obj_name)
                # self.collision_scene.update_group_blacklist(parent_group)
                # self.collision_scene.blacklist_inter_group_collisions()
            except:
                logging.loginfo('caught invalid object')

    @profile
    def update(self):
        #self.poll()
        return Status.SUCCESS
