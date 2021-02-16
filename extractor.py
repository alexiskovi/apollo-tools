from modules.canbus.proto import chassis_pb2
from modules.localization.proto import localization_pb2
from modules.localization.proto import gps_pb2
from modules.drivers.gnss.proto import gnss_best_pose_pb2
from modules.drivers.gnss.proto import imu_pb2
from modules.control.proto import control_cmd_pb2
from modules.planning.proto import planning_pb2
from modules.routing.proto import routing_pb2
from modules.map.proto import map_pb2
from modules.perception.proto import perception_obstacle_pb2
from cyber_py3.record import RecordReader
import common.proto_utils as proto_utils

import sys
import os
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter


def parse_args():
    parser = ArgumentParser(description='Extract info from cyber bags', formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument('in_bag', help='Path to input bags', nargs='*')
    args = parser.parse_args()
    return args


localization = localization_pb2.LocalizationEstimate()
gnss = gnss_best_pose_pb2.GnssBestPose()
chassis = chassis_pb2.Chassis()
imu = imu_pb2.Imu()

args = parse_args()

loc_file = open("{}_loc_file.txt".format(args.in_bag[0].split('.')[0]), 'w')
gnss_file = open("{}_gnss_file.txt".format(args.in_bag[0].split('.')[0]), 'w')
chassis_file = open("{}_chassis_file.txt".format(args.in_bag[0].split('.')[0]), 'w')
imu_file = open("{}_imu_file.txt".format(args.in_bag[0].split('.')[0]), 'w')

loc_file.write("timestamp x y z\n")
chassis_file.write("timestamp speed_mps steering_percentage \n")
gnss_file.write("timestamp latitude longitude lat_std_dev lon_std_dev\n")
imu_file.write("timestamp a_x a_y a_z w_x w_y w_z \n")

for bag_names in args.in_bag:
    print("Start reading {}...".format(bag_names))
    reader = RecordReader(bag_names)
    for msg in reader.read_messages():

        if msg.topic == '/apollo/localization/pose':
            localization.ParseFromString(msg.message)
            loc_string = ""
            loc_string += "{} {} {} {}\n".format(str(localization.header.timestamp_sec), str(localization.pose.position.x), str(localization.pose.position.y), str(localization.pose.position.z))
            loc_file.write(loc_string)
        
        if msg.topic == '/apollo/canbus/chassis':
            chassis.ParseFromString(msg.message)
            chassis_string = ""
            chassis_string += "{} {} {}\n".format(str(chassis.header.timestamp_sec), str(chassis.speed_mps), str(chassis.steering_percentage))
            chassis_file.write(chassis_string)
        
        if msg.topic == '/apollo/sensor/gnss/best_pose':
            gnss.ParseFromString(msg.message)
            gnss_string = ""
            gnss_string += "{} {} {} {} {}\n".format(str(gnss.header.timestamp_sec), str(gnss.latitude), str(gnss.longitude), str(gnss.latitude_std_dev), str(gnss.longitude_std_dev))
            gnss_file.write(gnss_string)
        
        if msg.topic == '/apollo/sensor/gnss/imu':
            imu.ParseFromString(msg.message)
            imu_string = ""
            imu_string += "{} {} {} {} {} {} {}\n".format(str(imu.header.timestamp_sec), str(imu.linear_acceleration.x), str(imu.linear_acceleration.y), str(imu.linear_acceleration.z), str(imu.angular_velocity.x), str(imu.angular_velocity.y), str(imu.angular_velocity.z))
            imu_file.write(imu_string)

loc_file.close()
gnss_file.close()
imu_file.close()
chassis_file.close()