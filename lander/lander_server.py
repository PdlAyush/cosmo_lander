#!/usr/bin/env python3
import rclpy
import threading
import json, os

from rclpy.node import Node
from flask import Flask, request
from lander.msg import Coordinate, PointOfInterest, RegionOfInterest


class LanderServer(Node):

    def __init__(self):
        super().__init__('landerserver')

        # Creating a flask server
        self.app = Flask(__name__)
        self.app.add_url_rule('/rois', 'rois', self.process_request, methods=['GET'])
        self.app.add_url_rule('/pois', 'pois', self.process_request, methods=['GET'])
        self.app.add_url_rule('/add-roi', 'add-roi', self.process_request, methods=['POST'])
        self.app.add_url_rule('/add-poi', 'add-poi', self.process_request, methods=['POST'])

        # Creating publishers
        self.poi_publisher_ = self.create_publisher(PointOfInterest,'lander/pois', 10)
        self.roi_publisher_ = self.create_publisher(RegionOfInterest, 'lander/rois', 10)

        

    def process_request(self):
        endpoint = request.endpoint

        if endpoint == 'rois':
            script_directory = os.path.dirname(os.path.abspath(__file__))
            file_path = os.path.join(script_directory, 'roi.json')

            with open(file_path, 'r') as jsonFile:
                roi_data = json.load(jsonFile)

            return json.dumps(roi_data)
        
        elif endpoint == 'pois':
            script_directory = os.path.dirname(os.path.abspath(__file__))
            file_path = os.path.join(script_directory, 'poi.json')

            with open(file_path, 'r') as jsonFile:
                poi_data = json.load(jsonFile)

            return json.dumps(poi_data)
        
        elif endpoint == 'add-roi':
            data = request.json
            script_directory = os.path.dirname(os.path.abspath(__file__))
            file_path = os.path.join(script_directory, 'roi.json')
            existing_data = []

            with open(file_path, 'r') as jsonFile:
                existing_data = json.load(jsonFile)
            
               
            # Converting dictionary to list
            if not isinstance(existing_data, list):
                existing_data = [existing_data] 
            
            existing_data.append(data)

            with open(file_path, 'w') as jsonFile:
                json.dump(existing_data, jsonFile)
            
            self.publish_roi(data)
            
            return 'ROI added successfully'
        
        elif endpoint == 'add-poi':
            data = request.json
            script_directory = os.path.dirname(os.path.abspath(__file__))
            file_path = os.path.join(script_directory, 'poi.json')
            existing_data = []

            with open(file_path, 'r') as jsonFile:
                existing_data = json.load(jsonFile)
            
            # Converting dictionary to list
            if not isinstance(existing_data, list):
                existing_data = [existing_data] 
            
            existing_data.append(data)

            with open(file_path, 'w') as jsonFile:
                json.dump(existing_data, jsonFile)
            
            self.publish_poi(data)

            return 'POI added successfully'
    
    def publish_roi(self, data):
        roi_msg = RegionOfInterest()
        roi_msg.region_id = data['RegionID']

        bounds = []
        for bound in data['Bounds']:
            coordinates = Coordinate()
            coordinates.latitude = bound['y']
            coordinates.longitude = bound['x']
            bounds.append(coordinates)
        roi_msg.bounds = bounds

        rover_path = []
        for rpath in data['RoverPath']:
            coordinates = Coordinate()
            coordinates.latitude = rpath['y']
            coordinates.longitude = rpath['x']
            rover_path.append(coordinates)
        roi_msg.rover_path = rover_path

        uav_path = []
        for upath in data['UAVPath']:
            coordinates = Coordinate()
            coordinates.latitude = upath['y']
            coordinates.longitude = upath['x']
            uav_path.append(coordinates)
        roi_msg.uav_path = uav_path

        self.roi_publisher_.publish(roi_msg)

    def publish_poi(self, data):
        poi_msg = PointOfInterest()
        poi_msg.point_id = data['PID']
        poi_msg.region_id = data['RegionID']

        coordinates = Coordinate()
        coordinates.latitude = data['Coordinates']['y']
        coordinates.longitude = data['Coordinates']['x']
        poi_msg.coordinates = coordinates

        poi_msg.task = data['Task']
        poi_msg.status = data['Status']

        self.poi_publisher_.publish(poi_msg)
    
    # Run the server:
    def run(self):
        self.app.run(host='0.0.0.0', port =5000, threaded = True)

def main(args=None):
    rclpy.init(args=args)
    landerserver = LanderServer()

    # Run the flask server in a seperate thread 
    flask_thread = threading.Thread(target=landerserver.run)
    flask_thread.start()

    # Run the ros node in a seperate thread
    ros_thread = threading.Thread(target =rclpy.spin, args=(landerserver,))
    ros_thread.start()

    #Join the threads
    flask_thread.join()
    ros_thread.join()

    landerserver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
