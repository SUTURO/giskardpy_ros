from typing import Tuple

from geometry_msgs.msg import PoseStamped, PointStamped, Vector3

from giskardpy.goals.goal import Goal


class GraspBox(Goal):
    def __init__(self,
                 box_pose: PoseStamped,
                 tip_link: str,
                 box_x_length: float,
                 box_y_length: float,
                 box_z_length: float):
        super().__init__()
        giskard_link_name = self.world.get_link_name(tip_link)
        root_link = self.world.root_link_name
        map_box_pose = self.transform_msg('map', box_pose)

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()


class OpenDrawer(Goal):
    def __init__(self,
                 knob_pose: PointStamped,
                 direction: Vector3,
                 distance: float):
        pass

    def make_constraints(self):
        pass

    def __str__(self) -> str:
        return super().__str__()
