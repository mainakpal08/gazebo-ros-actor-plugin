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

def get_waypoints_array_with_position_orientation(path, waypoints):
    """
    Given a list of waypoint indices and a list of waypoints,
    return the corresponding waypoints' positions and orientations as an array.
    """
    return [{'index': idx, 
             'position': waypoints[idx]['position'], 
             'orientation': waypoints[idx]['orientation']} 
            for idx in path]

def generate_path(graph, waypoints, start, end):
    """
    Generate a path from start to end node.
    """
    path = nx.shortest_path(graph, source=start, target=end)
    return get_waypoints_array_with_position_orientation(path, waypoints)

def publish_path(start, end, path_topic):
    # Load the yaml file
    file_path = '/home/exouser/simulation_ws/src/waypoints_visualizer/wp.yaml'  # Update this path to your wp.yaml file
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)

    # Extract waypoints and edges
    waypoints = data['waypoints']
    edges = data['edges']

    # Create a graph
    G = nx.Graph()

    # Add nodes with positions
    for idx, wp in enumerate(waypoints):
        G.add_node(idx, pos=(wp['position']['x'], wp['position']['y']))

    # Add edges
    for edge in edges:
        if edge['start'] != edge['end']:
            G.add_edge(edge['start'], edge['end'])

    # Generate the path
    path_data = generate_path(G, waypoints, start, end)

    # Initialize the node with a unique name
    node_name = 'path_publisher_node_' + str(uuid.uuid4())
    rospy.init_node(node_name, anonymous=True)
    
    # Create the publisher with the specified topic and message type "Path"
    pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
    
    # Create the Path message
    path_msg = Path()
    
    # Set the header for the Path message
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "map"
    
    # Create the PoseStamped messages for each waypoint
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
    
    # Publish the Path message to the specified topic
    rate = rospy.Rate(10)  # Publish at 10 Hz
    pub.publish(path_msg)
    print(f'Published path on {path_topic}:', path_msg)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(description="Publish a path from start to end node")
        parser.add_argument('start', type=int, help='Start node index')
        parser.add_argument('end', type=int, help='End node index')
        parser.add_argument('actor', type=int, choices=[1, 2], help='Actor number (1 or 2)')
        args = parser.parse_args()

        path_topic = f'/cmd_path_actor{args.actor}'
        publish_path(args.start, args.end, path_topic)
    except rospy.ROSInterruptException:
        pass
