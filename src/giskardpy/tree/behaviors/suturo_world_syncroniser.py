from py_trees import Status

from geometry_msgs.msg import Pose

from giskardpy.model.utils import make_world_body_box, make_world_body_cylinder, make_world_body_sphere
from giskardpy.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils import logging

from importlib import import_module

try:
    rosprolog_client = import_module("rosprolog_client")
    Prolog = rosprolog_client.Prolog
    prolog_exists = True
except:
    prolog_exists = False

# Debugging
# import pydevd_pycharm
# pydevd_pycharm.settrace('localhost', port=1234, stdoutToServer=True, stderrToServer=True, suspend=False)

#FiXME delete if not used at the end of project
class SuturoWorldSynchroniser(GiskardBehavior):
    """
    Receive updates and add objects newly added into the world to synchronise world states.

    Note:
        Only works when rosprolog is installed as well. Skips update otherwise.
        Will also stop/continue whenever rosprolog is stopped/continued.
    """
    if prolog_exists:
        prolog: Prolog

    last_update: float
    receiving_updates: bool

    @profile
    def __init__(self, name=None,
                 name_space='rosprolog'):
        if name is None:
            name = 'suturo knowledge world poller'
        super().__init__(name)
        self.name_space = name_space

    @profile
    def setup(self, timeout=0.0):
        if prolog_exists:
            self.prolog = Prolog(name_space=self.name_space,
                                 wait_for_services=False)
        else:
            logging.logwarn('Knowledge synchronizer: Prolog not found. Can\'t receive updates.')
        self.last_update = 0.0
        self.receiving_updates = True
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
        try:
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
                    obj_type = obj[0]

                    # Create object body
                    if obj_type == 'box':
                        obj_body = make_world_body_box(obj[1], obj[2], obj[3])
                    elif obj_type == 'cylinder':
                        obj_body = make_world_body_cylinder(obj[2], obj[1])
                    elif obj_type == 'sphere':
                        obj_body = make_world_body_sphere(obj[1])
                    else:
                        logging.loginfo('No object body to add')
                        continue

                    logging.loginfo(f'added {obj_type}')

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
                    # parent_link = self.world.get_link_name(obj_parent_link, obj_parent_link_group)

                    self.world.add_world_body(obj_name, obj_body, obj_pose, parent_link)

                    # code to update self collision matrix when objects are attached to the robot
                    # parent_group = self.world.get_parent_group_name(obj_name)
                    # self.collision_scene.update_group_blacklist(parent_group)
                    # self.collision_scene.blacklist_inter_group_collisions()
                except:
                    logging.loginfo('caught invalid object')
            if not self.receiving_updates:
                logging.logwarn('Knowledge synchronizer: connected')
                self.receiving_updates = True

        except:
            # Was not able to receive updates
            # Can happen due to missing connection
            if self.receiving_updates:
                logging.logwarn('Knowledge synchronizer: not connected')
            self.receiving_updates = False

    @profile
    def update(self):
        if prolog_exists:
            #self.poll()
            pass
        return Status.SUCCESS
