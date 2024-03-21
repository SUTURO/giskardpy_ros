import rospy
from controller_manager_msgs.srv import ListControllers, ListControllersResponse, SwitchController, \
    SwitchControllerResponse
from sensor_msgs.msg import Joy


class SwitchControllers:

    def __init__(self):
        rospy.init_node("~/switch_controllers")

        self.data = Joy()

        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        self.srv_list_con = rospy.ServiceProxy(name='/hsrb/controller_manager/list_controllers',
                                               service_class=ListControllers)
        rospy.wait_for_service('/hsrb/controller_manager/switch_controller')
        self.srv_switch_con = rospy.ServiceProxy(name='/hsrb/controller_manager/switch_controller',
                                                 service_class=SwitchController)

        self.sub_joy = rospy.Subscriber(name='/hsrb/joy', data_class=Joy, callback=self.testing_cb)

    def testing_cb(self, data: Joy):
        if data.buttons[7] == 1 and data.buttons[8] == 1 and self.check_controllers():
            self.switch_controls()

    def check_controllers(self):
        resp: ListControllersResponse = self.srv_list_con()
        needed_controllers = ['realtime_body_controller_real',
                              'arm_trajectory_controller',
                              'head_trajectory_controller']

        for controller in resp.controller:
            if controller.name in needed_controllers:
                needed_controllers.remove(controller.name)
            if len(needed_controllers) == 0:
                return True

        return False

    def switch_controls(self):
        resp: ListControllersResponse = self.srv_list_con()

        controller_dict = {controller.name: controller for controller in resp.controller}

        if (controller_dict['arm_trajectory_controller'].state == 'running'
                and controller_dict['head_trajectory_controller'].state == 'running'
                and (controller_dict['realtime_body_controller_real'].state == 'stopped'
                     or controller_dict['realtime_body_controller_real'].state == 'initialized')):
            print('test')
            start_con = ['realtime_body_controller_real']
            stop_con = ['arm_trajectory_controller', 'head_trajectory_controller']
            strictness: int = 2
            start_asap: bool = False
            timeout: float = 0.0
            resp: SwitchControllerResponse = self.srv_switch_con(start_con, stop_con, strictness, start_asap, timeout)

        elif ((controller_dict['arm_trajectory_controller'].state == 'stopped'
               or controller_dict['arm_trajectory_controller'].state == 'initialized')
              and (controller_dict['head_trajectory_controller'].state == 'stopped'
                   or controller_dict['head_trajectory_controller'].state == 'initialized')
              and controller_dict['realtime_body_controller_real'].state == 'running'):
            print('test2')
            start_con = ['arm_trajectory_controller', 'head_trajectory_controller']
            stop_con = ['realtime_body_controller_real']
            strictness = 2
            start_asap: bool = False
            timeout: float = 0.0
            resp: SwitchControllerResponse = self.srv_switch_con(start_con, stop_con, strictness, start_asap, timeout)
        else:
            print('test3')


if __name__ == '__main__':
    demo = SwitchControllers()

    rospy.spin()
