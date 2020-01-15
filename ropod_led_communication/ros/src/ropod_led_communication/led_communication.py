import rospy
from ropod_ros_msgs.msg import StateInfo, Status
from ropod_ros_msgs.msg import DockActionFeedback, GoToActionFeedback
from ropod_ros_msgs.msg import NavElevatorActionFeedback
from ropod_ros_msgs.msg import RoutePlannerActionGoal, RoutePlannerActionResult
from ropod_ros_msgs.msg import GoToActionGoal, GoToActionResult
from ropod_ros_msgs.msg import SmartWheelData
from maneuver_navigation.msg import Feedback
from ropod_led_communication.circuit_playground_comm import CircuitPlaygroundComm
from std_msgs.msg import String

class LEDCommunicator(object):
    def __init__(self):
        self.led_comm = CircuitPlaygroundComm()
        self.led_comm.on(self.led_comm.WHITE)
        rospy.sleep(1)
        self.led_comm.off()
        self.dock_progress_sub = rospy.Subscriber('task_progress_dock', DockActionFeedback, self.dock_progress_cb)
        #self.goto_progress_sub = rospy.Subscriber('task_progress_goto', GoToActionFeedback, self.goto_progress_cb)
        self.route_nav_feedback_sub = rospy.Subscriber('route_nav_feedback', Feedback, self.route_nav_feedack_cb)
        self.elevator_feedback_sub = rospy.Subscriber('task_progress_elevator', NavElevatorActionFeedback, self.elevator_feedack_cb)
        self.align_goal_sub = rospy.Subscriber('align_goal', GoToActionGoal, self.align_goal_cb)
        self.align_result_sub = rospy.Subscriber('align_result', GoToActionResult, self.align_result_cb)
        self.route_planner_goal_sub = rospy.Subscriber('route_planner_goal', RoutePlannerActionGoal, self.route_planner_goal_cb)
        self.route_planner_result_sub = rospy.Subscriber('route_planner_result', RoutePlannerActionResult, self.route_planner_result_cb)
        self.napoleon_state_sub = rospy.Subscriber('napoleon_state', String, self.napoleon_state_cb, queue_size=1)
        self.napoleon_result_sub = rospy.Subscriber('napoleon_result', GoToActionResult, self.napoleon_result_cb)
        #self.sw_ethercat_sub = rospy.Subscriber('ethercat_topic', SmartWheelData, self.ethercat_cb)
        self.wait_for_elevator_in_progress = False
        self.entering_in_progress = False

    def napoleon_result_cb(self, msg):
        self.led_comm.off()

    def napoleon_state_cb(self, msg):
        if msg.data == "CRUISING":
            self.led_comm.on(self.led_comm.GREEN)
        elif msg.data == "ENTRY_BEFORE_INTERSECTION":
            self.led_comm.on(self.led_comm.YELLOW)
        elif msg.data == "GOING_STRAIGHT_ON_INTERSECTION":
            self.led_comm.on(self.led_comm.GREEN)
        elif msg.data == "ACCELERATE_ON_INTERSECTION":
            self.led_comm.on(self.led_comm.GREEN)
        elif msg.data == "TIGHT_OVERTAKE":
            self.led_comm.blink(self.led_comm.YELLOW, 2)
        elif msg.data == "SPACIOUS_OVERTAKE":
            self.led_comm.blink(self.led_comm.YELLOW, 1)
        elif msg.data == "ALIGN_AXIS_AT_INTERSECTION":
            self.led_comm.on(self.led_comm.YELLOW)
        elif msg.data == "TURNING":
            self.led_comm.rotate(self.led_comm.YELLOW)
        else:
            self.led_comm.off()

    def dock_progress_cb(self, msg):
        if msg.feedback.feedback.status.module_code != Status.MOBIDIK_COLLECTION:
            return
        if msg.feedback.feedback.status.sm_state == 'LOOK_FOR_CART':
            self.led_comm.blink(self.led_comm.RED, 2)
        elif msg.feedback.feedback.status.status_code == Status.ACTIVE_SEARCH_MODE:
            rospy.sleep(2)
            self.led_comm.rotate(self.led_comm.WHITE)
        elif msg.feedback.feedback.status.status_code == Status.MOBIDIK_DETECTED:
            self.led_comm.blink(self.led_comm.GREEN, 5)
        elif msg.feedback.feedback.status.sm_state == 'GET_SETPOINT_IN_PRE_DOCK_AREA':
            pass
        elif msg.feedback.feedback.status.sm_state == 'GO_TO_PRE_DOCK_SETPOINT':
            self.led_comm.blink_forever(self.led_comm.YELLOW)
        elif msg.feedback.feedback.status.sm_state == 'ALIGN_AND_APPROACH_CART':
            self.led_comm.blink_and_buzz(self.led_comm.WHITE)
        elif msg.feedback.feedback.status.sm_state == 'COUPLE_TO_CART':
            self.led_comm.blink(self.led_comm.GREEN, 5)
            rospy.sleep(2)
            self.led_comm.rotate(self.led_comm.WHITE)
        elif msg.feedback.feedback.status.sm_state == 'GET_SETPOINT_IN_POST_DOCK_AREA':
            self.led_comm.blink(self.led_comm.GREEN, 5)
            rospy.sleep(2)
            self.led_comm.rotate(self.led_comm.WHITE)
        elif msg.feedback.feedback.status.sm_state == 'GO_TO_POST_DOCK_SETPOINT':
            pass
        elif msg.feedback.feedback.status.status_code == Status.DOCKING_SEQUENCE_SUCCEEDED:
            self.led_comm.blink(self.led_comm.GREEN, 10)
        elif msg.feedback.feedback.status.status_code == Status.DOCKING_SEQUENCE_FAILED:
            self.led_comm.blink(self.led_comm.RED, 10)
        else:
            self.led_comm.off()

    def goto_progress_cb(self, msg):
        if msg.feedback.feedback.status.module_code != Status.ROUTE_NAVIGATION and \
           msg.feedback.feedback.status.module_code != Status.UNDEFINED_MODULE:
            print("wrong module ")
            return
        if msg.feedback.feedback.status.status_code == Status.GOAL_REACHED:
            self.led_comm.blink(self.led_comm.GREEN, 5)
        elif msg.feedback.feedback.status.status_code == Status.GOAL_NOT_REACHABLE:
            self.led_comm.blink(self.led_comm.RED, 5)
        elif msg.feedback.feedback.status.status_code == Status.NAVIGATION_IN_PROGRESS:
            progress = float(msg.feedback.feedback.sequenceNumber) * 100.0 / msg.feedback.feedback.totalNumber
            progress = int(progress)
            self.led_comm.set_progress(progress)

    def route_nav_feedack_cb(self, msg):
        if msg.status == Feedback.FAILURE_OBSTACLES:
            self.led_comm.blink_forever(self.led_comm.YELLOW)

    def elevator_feedack_cb(self, msg):
        rotate = False
        blink = False
        if msg.feedback.feedback.status.status_code == Status.WAITING and \
                not self.wait_for_elevator_in_progress:
            rotate = True
            self.wait_for_elevator_in_progress = True
        elif msg.feedback.feedback.status.status_code == Status.GOAL_REACHED:
            self.led_comm.blink(self.led_comm.GREEN, 5)
            rotate = False
            self.wait_for_elevator_in_progress = False
            self.entering_in_progress = False
            blink = False
            rospy.sleep(2)
        elif msg.feedback.feedback.status.status_code == Status.ENTERING and \
                not self.entering_in_progress:
            self.entering_in_progress = True
            blink = True
        elif msg.feedback.feedback.status.status_code == Status.ELEVATOR_ENTERING_FAILED:
            self.led_comm.blink(self.led_comm.RED, 15)
            rotate = False
            self.wait_for_elevator_in_progress = False
            self.entering_in_progress = False
            blink = False

        if rotate:
            self.led_comm.rotate(self.led_comm.WHITE)
        elif blink:
            self.led_comm.blink_and_buzz(self.led_comm.WHITE)
    def align_goal_cb(self, msg):
        self.led_comm.blink_forever(self.led_comm.YELLOW)
    def align_result_cb(self, msg):
        if msg.result.success == True:
            self.led_comm.blink(self.led_comm.GREEN, 5)
        else:
            self.led_comm.blink(self.led_comm.RED, 5)

    def route_planner_goal_cb(self, msg):
        self.led_comm.rotate(self.led_comm.YELLOW)
    def route_planner_result_cb(self, msg):
        if len(msg.result.areas) > 0:
            self.led_comm.blink(self.led_comm.GREEN, 5)
        else:
            self.led_comm.blink(self.led_comm.RED, 5)

    def ethercat_cb(self, msg):
        self.wheel_disabled = False
        disabled_wheels = []
        for i in range(4):
            status = msg.sensors[i].status1
            if (status != 63.0):
                self.wheel_disabled = True
                disabled_wheels.append(i)
        if (self.wheel_disabled):
            self.led_comm.on(self.led_comm.RED)
        else:
            self.led_comm.on(self.led_comm.GREEN)


