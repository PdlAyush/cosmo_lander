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

        # Creating timer call-back functions to publish data periodically

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

            with open(file_path, 'r') as jsonFile:
                existing_data = json.load(jsonFile)
            
            existing_data.append(data)

            with open(file_path, 'w') as jsonFile:
                json.dump(existing_data, jsonFile)
            
            self.publish_roi(data)
            
            return 'ROI added successfully'
        
        elif endpoint == 'add-poi':
            data = request.json
            script_directory = os.path.dirname(os.path.abspath(__file__))
            file_path = os.path.join(script_directory, 'poi.json')

            with open(file_path, 'r') as jsonFile:
                existing_data = json.load(jsonFile)
            
            existing_data.append(data)

            with open(file_path, 'w') as jsonFile:
                json.dump(existing_data, jsonFile)
            
            self.publish_poi(data)

            return 'POI added successfully'
    
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
