#!/usr/bin/env python
import rospy
from controller_manager_msgs.srv import ListControllers, ListControllersResponse, SwitchController, \
    SwitchControllerResponse
from typing import List
from sensor_msgs.msg import Joy


class SwitchControllers:

    def __init__(self,
                 list_con_srv: str,
                 switch_con_srv: str,
                 con_list1: List[str],
                 con_list2: List[str],
                 joy_topic: str,
                 buttons: List[int]):
        """
        @param list_con_srv Name of the Service for ListControllers
        @param switch_con_srv Name of the Service for SwitchController
        @param con_list1 List of controllers to start/stop
        @param con_list2 List of controllers to stop/start, con_list2 start, when con_list1 is stopped and vice versa
        @param joy_topic Topic where joy is published to, to get the correct buttons
        @param buttons Indexes of the buttons used for the button-combination
        """
        rospy.init_node("giskard_joy_switch_controllers")

        self.data = Joy()
        self.con_list1 = con_list1
        self.con_list2 = con_list2
        if len(buttons) <= 0:
            self.buttons = [0]
        else:
            self.buttons = buttons

        self.last_change: float = 0
        rospy.wait_for_service(list_con_srv)
        self.srv_list_con = rospy.ServiceProxy(name=list_con_srv,
                                               service_class=ListControllers)
        rospy.wait_for_service(switch_con_srv)
        self.srv_switch_con = rospy.ServiceProxy(name=switch_con_srv,
                                                 service_class=SwitchController)

        self.sub_joy = rospy.Subscriber(name=joy_topic, data_class=Joy, callback=self.testing_cb)

    def testing_cb(self, data: Joy):
        time_diff = data.header.stamp.secs - self.last_change
        if (all(data.buttons[i] == 1 for i in self.buttons if len(data.buttons) > i)
                and self.check_controllers() and time_diff > 5.0):
            self.last_change = data.header.stamp.secs
            self.switch_controls()

    def check_controllers(self):
        resp: ListControllersResponse = self.srv_list_con()
        needed_controllers = self.con_list2 + self.con_list1

        for controller in resp.controller:
            if controller.name in needed_controllers:
                needed_controllers.remove(controller.name)
            if len(needed_controllers) == 0:
                return True

        return False

    def switch_controls(self):
        resp: ListControllersResponse = self.srv_list_con()

        controller_dict = {controller.name: controller for controller in resp.controller}

        strictness: int = 2
        start_asap: bool = False
        timeout: float = 0.0

        if (all(controller_dict[con].state == 'stopped' or controller_dict[con].state == 'initialized' for con in
                self.con_list1) and all(controller_dict[con].state == 'running' for con in self.con_list2)):
            start_con = self.con_list1
            stop_con = self.con_list2
            resp: SwitchControllerResponse = self.srv_switch_con(start_con, stop_con, strictness, start_asap, timeout)

        elif (all(controller_dict[con].state == 'stopped' or controller_dict[con].state == 'initialized' for con in
                  self.con_list2) and all(controller_dict[con].state == 'running' for con in self.con_list1)):
            start_con = self.con_list2
            stop_con = self.con_list1
            resp: SwitchControllerResponse = self.srv_switch_con(start_con, stop_con, strictness, start_asap, timeout)


if __name__ == '__main__':
    demo = SwitchControllers(list_con_srv='/hsrb/controller_manager/list_controllers',
                             switch_con_srv='/hsrb/controller_manager/switch_controller',
                             con_list1=['arm_trajectory_controller',
                                        'head_trajectory_controller'],
                             con_list2=['realtime_body_controller_real'],
                             joy_topic='/hsrb/joy',
                             buttons=[7, 8])

    rospy.spin()
