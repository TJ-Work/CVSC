import py_trees

from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.scenarios.basic_scenario import *
from srunner.tools.scenario_helper import *

CONTROL_ASSESSMENT = [
    "ControlAssessment"
]


class ControlAssessment(BasicScenario):

    """
    Implementation of "Control Loss Vehicle" (Traffic Scenario 01)

    This is a single ego vehicle scenario
    """

    category = "ControlAssessment"

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, timeout=35 * 60):
        """
        Setup all relevant parameters and create scenario
        """
        self.config = config
        self.debug = debug_mode

        self.timeout = timeout  # Timeout of scenario in seconds

        self.route = config.route.data

        super(ControlAssessment, self).__init__("ControlAssessment",
                                                 ego_vehicles,
                                                 config,
                                                 world,
                                                 debug_mode,
                                                 terminate_on_failure=True,
                                                 criteria_enable=True)

    def _create_behavior(self):
        """
        Basic behavior do nothing, i.e. Idle
        """

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        
        global_publish = PublishGlobalPlan("ros_behavior", self.route)
        complete = RouteCompletionTest(self.ego_vehicles[0], self.route, DISTANCE_THRESHOLD = 1)

        sequence.add_child(global_publish)
        sequence.add_child(complete)
        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        for ego_vehicle in self.ego_vehicles:
            collision_criterion = CollisionTest(ego_vehicle)
            inroute = InRouteTest(ego_vehicle, 20, self.route, 4) #argument offroad_max is useless
            
            criteria.append(collision_criterion)
            criteria.append(inroute)
        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
