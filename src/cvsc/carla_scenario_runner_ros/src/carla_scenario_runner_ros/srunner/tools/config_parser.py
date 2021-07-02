#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides a parser for scenario configuration files
"""

import glob
import os
import rospkg
import xml.etree.ElementTree as ET

import carla
from agents.navigation.local_planner import RoadOption


class RouteConfiguration(object):

    """
    This class provides the basic  configuration for a route
    """

    def __init__(self, route=None):
        if route is not None:
            self.data = route
        else:
            self.data = None

    def parse_xml(self, node):
        """
        Parse route config XML
        """
        self.data = []

        for waypoint in node.iter("waypoint"):
            x = float(waypoint.attrib.get('x', 0))
            y = float(waypoint.attrib.get('y', 0))
            z = float(waypoint.attrib.get('z', 0))
            c = waypoint.attrib.get('connection', '.VOID')
            connection = RoadOption[c.split('.')[1]]

            self.data.append((carla.Location(x, y, z), connection))


class TargetConfiguration(object):

    """
    This class provides the basic  configuration for a target location
    """

    transform = None

    def __init__(self, node):
        pos_x = float(node.attrib.get('x', 0))
        pos_y = float(node.attrib.get('y', 0))
        pos_z = float(node.attrib.get('z', 0))

        self.transform = carla.Transform(carla.Location(x=pos_x, y=pos_y, z=pos_z))


class ActorConfigurationData(object):

    """
    This is a configuration base class to hold model and transform attributes
    """

    def __init__(self, model, transform, rolename='other', autopilot=False, random=False, amount=1):
        self.model = model
        self.rolename = rolename
        self.transform = transform
        self.autopilot = autopilot
        self.random_location = random
        self.amount = amount


class ActorConfiguration(ActorConfigurationData):

    """
    This class provides the basic actor configuration for a
    scenario:
    - Location and rotation (transform)
    - Model (e.g. Lincoln MKZ2017)
    """

    def __init__(self, node, rolename):

        pos_x = float(node.attrib.get('x', 0))
        pos_y = float(node.attrib.get('y', 0))
        pos_z = float(node.attrib.get('z', 0))
        yaw = float(node.attrib.get('yaw', 0))

        random_location = False
        if 'random_location' in node.keys():
            random_location = True

        autopilot = False
        if 'autopilot' in node.keys():
            autopilot = True

        amount = 1
        if 'amount' in node.keys():
            amount = int(node.attrib['amount'])

        super(ActorConfiguration, self).__init__(node.attrib.get('model', 'vehicle.*'),
                                                 carla.Transform(carla.Location(x=pos_x, y=pos_y, z=pos_z),
                                                 carla.Rotation(yaw=yaw)),
                                                 node.attrib.get('rolename', rolename),
                                                 autopilot, random_location, amount)


class ScenarioConfiguration(object):

    """
    This class provides a basic scenario configuration incl.:
    - configurations for all actors
    - town, where the scenario should be executed
    - name of the scenario (e.g. ControlLoss_1)
    - type is the class of scenario (e.g. ControlLoss)
    """

    trigger_points = []
    ego_vehicles = []
    other_actors = []
    town = None
    name = None
    type = None
    target = None
    route = None


def parse_scenario_configuration(scenario_config_file, scenario_name):
    """
    Parse scenario configuration file and provide a list of
    ScenarioConfigurations @return

    If scenario_name starts with "group:" all scenarios within
    the config file will be returned. Otherwise only the scenario,
    that matches the scenario_name.

    return: scenario_configurations:  a list of ScenarioConfiguration objects
    ScenarioConfiguration members:
    string: 
        town, name, type
    list: 
        other_actors(ActorConfiguration("scenario")), 
        ego_vehicles(ActorConfiguration("hero"))
        trigger_points: (starting points of ego_vehicles)
    RouteConfiguration:
        route

    """

    single_scenario_only = True
    if scenario_name.startswith("group:"):
        single_scenario_only = False
        scenario_name = scenario_name[6:]

    tree = ET.parse(scenario_config_file)

    scenario_configurations = []

    for scenario in tree.iter("scenario"):

        new_config = ScenarioConfiguration()
        new_config.town = scenario.attrib.get('town', None)
        new_config.name = scenario.attrib.get('name', None)
        new_config.type = scenario.attrib.get('type', None)
        new_config.other_actors = []
        new_config.ego_vehicles = []
        new_config.trigger_points = []

        for ego_vehicle in scenario.iter("ego_vehicle"):
            new_config.ego_vehicles.append(ActorConfiguration(ego_vehicle, 'ego_vehicle'))
            new_config.trigger_points.append(new_config.ego_vehicles[-1].transform)

        for target in scenario.iter("target"):
            new_config.target = TargetConfiguration(target)

        for route in scenario.iter("route"):
            route_conf = RouteConfiguration()
            route_conf.parse_xml(route)
            new_config.route = route_conf

        for other_actor in scenario.iter("other_actor"):
            new_config.other_actors.append(ActorConfiguration(other_actor, 'scenario'))
        
        for object_ in scenario.iter("object"):
            new_config.other_actors.append(ActorConfiguration(object_, 'other_object'))

        if single_scenario_only:
            if new_config.name == scenario_name:
                scenario_configurations.append(new_config)
        else:
            scenario_configurations.append(new_config)

    return scenario_configurations


def get_list_of_scenarios(config_file_name):
    """
    Parse *all* config files and provide a list with all scenarios @return
    """
    rp = rospkg.RosPack()
    current_pkg_name = "carla_scenario_runner_ros"
    path = rp.get_path(current_pkg_name) + "/src/{}".format(current_pkg_name)
    list_of_config_files = glob.glob("{}/srunner/configs/*.xml".format(path))

    if config_file_name != '':
        list_of_config_files.append(config_file_name)

    scenarios = []
    for file_name in list_of_config_files:
        tree = ET.parse(file_name)
        for scenario in tree.iter("scenario"):
            scenarios.append(scenario.attrib.get('name', None))

    return scenarios


def find_scenario_config(scenario_name, config_file_name):
    """
    Parse *all* config files and find first match for scenario config

    return : config xml file name
    """

    rp = rospkg.RosPack()
    current_pkg_name = "carla_scenario_runner_ros"
    path = rp.get_path(current_pkg_name) + "/src/{}".format(current_pkg_name)
    list_of_config_files = glob.glob("{}/srunner/configs/*.xml".format(path))
    if config_file_name != '': # default config_file_name = ""
        list_of_config_files.append(config_file_name)

    if scenario_name.startswith("group:"):
        scenario_name = scenario_name[6:]

        for file_name in list_of_config_files:
            tree = ET.parse(file_name) # using element tree to parse XML
            for scenario in tree.iter("scenario"):
                if scenario.attrib.get('type', None) == scenario_name:
                    return file_name

    else:
        for file_name in list_of_config_files:
            tree = ET.parse(file_name)
            for scenario in tree.iter("scenario"):
                if scenario.attrib.get('name', None) == scenario_name:
                    return file_name

    return None
