import os
import signal
import time
import math
import json

from cyber_py3 import cyber
from cyber_py3 import cyber_time
import common.proto_utils as proto_utils
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.drivers.gnss.proto.gnss_best_pose_pb2 import GnssBestPose
from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint, RoutingResponse
from modules.dreamview.proto.hmi_status_pb2 import HMIStatus
from modules.control.proto.control_cmd_pb2 import ControlCommand
from modules.routing.proto.poi_pb2 import POI

import map_lib

class ApolloAutoAPI:

    def __init__(self):

        cyber.init()
        self.api_node = cyber.Node("base_api")

        self.loc = {"x": 0.0, "y": 0.0, "z": 0.0, "heading": 0.0}
        self.veh = {"speed": 0.0, "steering": 0.0, "throttle": 0.0, "brake": 0.0}
        self.gps = {"latitude": 0.0, "longitude": 0.0}

        self.routing_writer = self.api_node.create_writer('/apollo/routing_request', RoutingRequest)
        self.control_writer = self.api_node.create_writer('/apollo/control', ControlCommand)
        self.api_node.create_reader("/apollo/localization/pose", LocalizationEstimate, self._localization_callback)
        self.api_node.create_reader("/apollo/canbus/chassis", Chassis, self._canbus_callback)
        self.api_node.create_reader("/apollo/sensor/gnss/best_pose", GnssBestPose, self._gnss_callback)
        self.api_node.create_reader("/apollo/routing_response", RoutingResponse, self._routing_callback)
        self.api_node.create_reader("/apollo/hmi/status", HMIStatus, self._hmi_callback)

        self.routing_info = {"length": 0}
        self.hmi = {
            "map": "",
            "car": "",
            "mode": ""
        }
    
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
    
    def _routing_callback(self, routing_msg):
        self.routing_info["length"] = routing_msg.measurement.distance
        lanes_count = len(routing_msg.road)
        lanes = [routing_msg.road[i].passage[0].segment[0].id for i in range(lanes_count)]
        first_point = [routing_msg.road[i].passage[0].segment[0].start_s for i in range(lanes_count)]
        last_point = [routing_msg.road[i].passage[0].segment[0].start_s for i in range(lanes_count)]
        roads = {}
        
        for i in range(lanes_count):
            roads[lanes[i]] = [first_point[i], last_point[i]]

        self.routing_response = [
            self.hmi["map"],
            roads,
            [[routing_msg.routing_request.waypoint[0].pose.x, routing_msg.routing_request.waypoint[0].pose.y], [routing_msg.routing_request.waypoint[1].pose.x, routing_msg.routing_request.waypoint[1].pose.y]]
        ]
    
    def _hmi_callback(self, hmi_msg):
        self.hmi = {
            "map": hmi_msg.current_map.lower().replace(" ", "_"),
            "car": hmi_msg.current_vehicle,
            "mode": hmi_msg.current_mode
        }
    
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
        poi = POI()

        try_count = 0
        while self.hmi["map"] == "" and try_count < 10:
            time.sleep(0.2)
            try_count += 1

        if try_count == 10:
            return {"data": {"poi": []}}
        try:
            proto_utils.get_pb_from_text_file("/apollo/modules/map/data/{}/default_end_way_point.txt".format(self.hmi["map"]), poi)
        except:
            return {"data": {"poi": []}}
        poi_list = []
        for landmarks in poi.landmark:
            for wp in landmarks.waypoint:
                poi_list.append({"name": wp.id, "position": {"latitude": wp.pose.x, "longitude": wp.pose.y}})
        res = {"data": {"poi": poi_list}}
        return res

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
        self.routing_response = []
        return self.get_routing_response()
    
    def get_routing_response(self):
        try_count = 0
        while self.routing_response == [] and try_count < 10:
            time.sleep(0.2)
            try_count += 1

        if try_count == 10:
            return {"status": 1, "statusText": "Routing timeout", "data": {}}
        else:
            routing_points = map_lib.get_points(self.routing_response)
            return {"status": 0, "statusText": "OK", "data": {"points": routing_points, "estimated_time": 100, "path_length": self.routing_info["length"]}}

    def start_autonomous_mode(self):
        controlcmd = ControlCommand()
        controlcmd.header.module_name = "control"
        controlcmd.header.timestamp_sec = cyber_time.Time.now().to_sec()
        controlcmd.pad_msg.action = 2
        self.control_writer.write(controlcmd)
        return {"data": {"info": "OK"}}

    def stop_autonomous_mode(self):
        controlcmd.pad_msg.action = 1
        controlcmd.throttle = 0
        controlcmd.brake = 0
        self.control_writer.write(controlcmd)
        return {"data": {"info": "OK"}}
    
    def _kill(self):
        cyber.shutdown()
        os.kill(os.getpid(), signal.SIGTERM)