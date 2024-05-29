#!/usr/bin/env python3
import math
import rospy
import argparse
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
import networkx as nx
import yaml
import uuid
import time
import threading

LINEAR_VELOCITY = 1.0
START_DELAY = 20.0

def get_waypoints_array_with_position_orientation(path, waypoints):
    return [{'index': idx, 
             'position': waypoints[idx]['position'], 
             'orientation': waypoints[idx]['orientation']} 
            for idx in path]

def generate_path(graph, waypoints, start, end):
    path = nx.shortest_path(graph, source=start, target=end)
    return get_waypoints_array_with_position_orientation(path, waypoints)

def calculate_distance(pos1, pos2):
    return math.sqrt((pos1['x'] - pos2['x'])**2 + (pos1['y'] - pos2['y'])**2)

def calculate_duration(path_data):
    total_distance = 0.0
    for i in range(len(path_data) - 1):
        pos1 = path_data[i]['position']
        pos2 = path_data[i + 1]['position']
        total_distance += calculate_distance(pos1, pos2)
    return total_distance / LINEAR_VELOCITY

def publish_path(start, end, path_topic):
    file_path = '/home/exouser/simulation_ws/src/waypoints_visualizer/wp.yaml'
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)

    waypoints = data['waypoints']
    edges = data['edges']

    G = nx.Graph()
    for idx, wp in enumerate(waypoints):
        G.add_node(idx, pos=(wp['position']['x'], wp['position']['y']))

    for edge in edges:
        if edge['start'] != edge['end']:
            G.add_edge(edge['start'], edge['end'])

    path_data = generate_path(G, waypoints, start, end)

    pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
    
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "map"
    
    for wp in path_data:
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(x=wp['position']['x'], y=wp['position']['y'], z=0)
        pose.pose.orientation = Quaternion(
            x=wp['orientation']['x'], 
            y=wp['orientation']['y'], 
            z=wp['orientation']['z'], 
            w=wp['orientation']['w']
        )
        path_msg.poses.append(pose)
    
    pub.publish(path_msg)
    print(f'Published path on {path_topic}:', path_msg)

    return calculate_duration(path_data)

def execute_courses_of_action(courses, path_topic):
    initial_setup_done = False
    for course in courses:
        start_node = course['start']
        end_node = course['end']
        wait_time = course['wait']
        duration = publish_path(start_node, end_node, path_topic)
        if not initial_setup_done:
            time.sleep(duration+wait_time+START_DELAY)
            initial_setup_done = True
        else:
            time.sleep(duration+wait_time)

def run_agent(courses, path_topic):
    execute_courses_of_action(courses, path_topic)

if __name__ == '__main__':
    rospy.init_node('multi_agent_path_publisher_node', anonymous=True)

    with open('/home/exouser/simulation_ws/src/waypoints_visualizer/trajectory.yaml', 'r') as file:
        courses = yaml.safe_load(file)

    # Define courses of action for both agents
    # courses_of_action_agent1 = [
    #     {'start': 0, 'end': 6, 'wait': 3},
    #     {'start': 6, 'end': 10, 'wait': 4},
    # ]

    # courses_of_action_agent2 = [
    #     {'start': 2, 'end': 22, 'wait': 5},
    #     {'start': 22, 'end': 11, 'wait': 5},
    # ]

    courses_of_action_agent1 = courses['agent1']
    courses_of_action_agent2 = courses['agent2']

    path_topic_agent1 = '/cmd_path_actor1'
    path_topic_agent2 = '/cmd_path_actor2'

    # Create threads for each agent
    thread_agent1 = threading.Thread(target=run_agent, args=(courses_of_action_agent1, path_topic_agent1))
    thread_agent2 = threading.Thread(target=run_agent, args=(courses_of_action_agent2, path_topic_agent2))

    # Start the threads
    thread_agent1.start()
    thread_agent2.start()

    # Wait for both threads to complete
    thread_agent1.join()
    thread_agent2.join()
