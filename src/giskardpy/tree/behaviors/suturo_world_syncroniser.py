import rospy
from geometry_msgs.msg import PoseStamped, Pose
from giskard_msgs.srv import UpdateWorldRequest, UpdateWorld
from py_trees import Status

from giskardpy.model.utils import make_world_body_box
from giskardpy.tree.behaviors.plugin import GiskardBehavior

from rosprolog_client import Prolog

import pydevd_pycharm
#pydevd_pycharm.settrace('localhost', port=1234, stdoutToServer=True, stderrToServer=True, suspend=False)


class SuturoWorldSynchroniser(GiskardBehavior):
    prolog: Prolog
    last_update: float
    _update_world_srv: rospy.ServiceProxy

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

        node_name = 'giskard'
        self._update_world_srv = rospy.ServiceProxy(f'{node_name}/update_world', UpdateWorld)
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
        updates = self.prolog.once(f"giskard_updates({self.last_update}, CurrentTime, Updates)")
        self.last_update = updates['CurrentTime']
        print(updates, flush=True)

        # TODO Add objects to giskard
        for element in updates['Updates']:
            #self.world.root_link_name
            #self.world.groups

            obj_name = str(element[2][0])
            obj_parent_link_group = ''
            obj_parent_link = ''
            obj_pose = Pose()
            obj_pose.position.x = element[2][1][0]
            obj_pose.position.y = element[2][1][1]
            obj_pose.position.z = element[2][1][2]
            obj_pose.orientation.x = element[2][2][0]
            obj_pose.orientation.y = element[2][2][1]
            obj_pose.orientation.z = element[2][2][2]
            obj_pose.orientation.w = element[2][2][3]

            timeout = 2.0
            obj = element[3]['term']
            print(obj)

            req = UpdateWorldRequest()

            req.group_name = obj_name
            req.operation = UpdateWorldRequest.ADD
            req.timeout = timeout
            if obj[0] == 'box':
                req.body = make_world_body_box(obj[1], obj[2], obj[3])
                print("box added")
            req.parent_link_group = obj_parent_link_group
            req.parent_link = self.world.get_link_name(obj_parent_link, obj_parent_link_group)
            req.pose = obj_pose

            self.world.add_world_body(obj_name, req.body, req.pose, req.parent_link)
            #self._update_world_srv.call(req)


    @profile
    def update(self):
        self.poll()
        return Status.SUCCESS
