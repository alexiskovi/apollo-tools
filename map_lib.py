import google.protobuf.text_format as text_format
import math
import sys
from modules.map.proto import map_pb2

"""
input1 = ["Gomentum", {"lane_15": [169.834280991, 176.645965576],
                      "lane_136": [0.00000000, 40.163932800],
                      "lane_12": [0.00000000, 17.581824940]},
         [[586564.300375366, 4207924.373527527], [586611.316199999, 4207939.602930394]]
        ]
input2 = ["Shalun", {"lane_69": [169.834280991, 176.645965576],
                      "lane_204": [0.00000000, 40.163932800],
                      "lane_14": [0.00000000, 40.163932800],
                      "lane_184": [0.00000000, 40.163932800],
                      "lane_18": [0.00000000, 17.581824940]},
         [[215722.456338501, 2546268.770828247], [215749.076301093, 2546182.430790525]]
        ]

input3 = ["Shalun", {"lane_69": [169.834280991, 176.645965576],
                      "lane_18": [0.00000000, 17.581824940]},
         [[215722.456338501, 2546268.770828247], [215749.076301093, 2546182.430790525]]
        ]

input4 = ["Shalun", {"lane_18": [0.00000000, 17.581824940]},
         [[215762.49227294922, 2546175.6635780334], [215749.076301093, 2546182.430790525]]
        ]
"""

def get_pb_from_text_file(filename, pb_value):
    """Get a proto from given text file."""
    with open(filename, 'r') as file_in:
        return text_format.Merge(file_in.read(), pb_value)

def distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x2-x1, 2)+math.pow(y2-y1, 2))

def get_points(input):
    folder = "/apollo/modules/map/data/"+input[0].lower()
    map_pb = map_pb2.Map()
    get_pb_from_text_file(folder+'/sim_map.txt', map_pb)
    lanes = input[1].copy()
    for lane_pb in map_pb.lane:
        if lane_pb.id.id in lanes.keys():
            lanes[lane_pb.id.id] = lane_pb
    
    if len(lanes)==1:
        lane = lanes[list(lanes.keys())[0]].central_curve.segment[0].line_segment
        min_distance_start = sys.float_info.max
        min_distance_end = sys.float_info.max
        points=[]
        for point in lane.point:
            cur_distance_start = distance(float(point.x), float(point.y), input[2][0][0], input[2][0][1])
            cur_distance_end = distance(float(point.x), float(point.y), input[2][1][0], input[2][1][1])
            if cur_distance_start < min_distance_start:
                points=[]
                min_distance_start=cur_distance_start
        
            if cur_distance_end < min_distance_end:
                min_distance_end=cur_distance_end
            elif cur_distance_end > min_distance_end:
                break
        
            points.append([point.x, point.y])
        
        return points


    start_lane = lanes[list(lanes.keys())[0]].central_curve.segment[0].line_segment
    min_distance = sys.float_info.max
    start_points=[]
    for point in start_lane.point:
        cur_distance = distance(float(point.x), float(point.y), input[2][0][0], input[2][0][1])
        if cur_distance < min_distance:
            start_points=[]
            min_distance=cur_distance
        start_points.append([point.x, point.y])

    middle_points=[]
    for key in list(lanes.keys())[1:-1]:
        lane = lanes[key]
        for point in lane.central_curve.segment[0].line_segment.point:
            middle_points.append([point.x, point.y])

    end_lane = lanes[list(lanes.keys())[-1]].central_curve.segment[0].line_segment
    point = end_lane.point[0]
    min_distance = sys.float_info.max
    end_points=[]
    for point in end_lane.point:
        cur_distance = distance(float(point.x), float(point.y), input[2][1][0], input[2][1][1])
        if cur_distance < min_distance:
            end_points.append([point.x, point.y])
            min_distance=cur_distance
        elif cur_distance > min_distance:
            break
    
    return start_points + middle_points + end_points

#print(get_points(input1))