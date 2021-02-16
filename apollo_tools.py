import os
import signal
import time
import math
import json

from cyber_py3 import cyber
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.drivers.gnss.proto.gnss_best_pose_pb2 import GnssBestPose
from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint

class ApolloAutoAPI:

    def __init__(self):

        cyber.init()
        self.api_node = cyber.Node("base_api")

        self.loc = {"x": 0.0, "y": 0.0, "z": 0.0, "heading": 0.0}
        self.veh = {"speed": 0.0, "steering": 0.0, "throttle": 0.0, "brake": 0.0}
        self.gps = {"latitude": 0.0, "longitude": 0.0}

        self.routing_writer = self.api_node.create_writer('/apollo/routing_request', RoutingRequest)

        self.api_node.create_reader("/apollo/localization/pose", LocalizationEstimate, self._localization_callback)
        self.api_node.create_reader("/apollo/canbus/chassis", Chassis, self._canbus_callback)
        self.api_node.create_reader("/apollo/sensor/gnss/best_pose", GnssBestPose, self._gnss_callback)
    
    def _localization_callback(self, localization_msg):
        self.loc["x"] = localization_msg.pose.position.x
        self.loc["y"] = localization_msg.pose.position.y
        self.loc["z"] = localization_msg.pose.position.z
        self.loc["heading"] = localization_msg.pose.heading / math.pi * 180.0
    
    def _canbus_callback(self, canbus_msg):
        self.veh["speed"] = canbus_msg.speed_mps
        self.veh["steering"] = canbus_msg.steering_percentage
        self.veh["throttle"] = canbus_msg.throttle_percentage
        self.veh["brake"] = canbus_msg.brake_percentage

    def _gnss_callback(self, gnss_msg):
        self.gps["longitude"] = gnss_msg.longitude
        self.gps["latitude"] = gnss_msg.latitude
    
    def get_agv_localization_wgs(self):
        res = {"data": {"position": {"latitude": self.gps["latitude"], "longitude": self.gps["longitude"]}, "heading": self.loc["heading"]}}
        return res
    
    def get_agv_localization_cartesian(self):
        res = {"data": {"position": {"x": self.loc["x"], "y": self.loc["y"], "z": self.loc["z"]}, "heading": self.loc["heading"]}}
        return res

    def get_agv_kinematic_state(self):
        res = {"data": {"state": {"speed": self.veh["speed"], "steering": self.veh["steering"], "throttle": self.veh["throttle"], "brake": self.veh["brake"]}}}
        return res

    def get_poi_list(self):
        example = {"data": {"poi": [{"name": "test_point", "position": {"latitude": 0.0, "longitude": 0.0}}]}}
        return example

    def get_hr_path_info(self):
        return 0

    def get_prediction_logs(self):
        return 0

    def send_routing_request(self, data):
        msg = RoutingRequest()
        time.sleep(0.5)
        msg.header.module_name = 'dreamview'
        msg.header.sequence_num = 0
        points = json.loads(data)
        for i in range(len(points)):
            waypoint = msg.waypoint.add()
            time.sleep(0.3)
            waypoint.pose.x = float(points[i][0])
            waypoint.pose.y = float(points[i][1])
        self.routing_writer.write(msg)
        return "0"

    def start_autonomous_mode(self):
        example = {"data": {"info": "OK"}}
        return example

    def stop_autonomous_mode(self):
        example = {"data": {"info": "OK"}}
        return example
    
    def _kill(self):
        cyber.shutdown()
        os.kill(os.getpid(), signal.SIGTERM)