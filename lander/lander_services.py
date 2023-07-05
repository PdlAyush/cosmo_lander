#!/usr/bin/env python3

from lander.srv import  GetPOI, GetROI, SetPOI
from lander.msg import Coordinate, PointOfInterest, RegionOfInterest

import rclpy
from rclpy.node import Node
import json
import os

class LanderServices(Node):

    def __init__(self):
        super().__init__('landerservices')
        self.srv = self.create_service(GetPOI, 'getPOI', self.getPOI_callback)
        self.srv = self.create_service(GetROI,'getROI', self.getROI_callback)
        self.srv = self.create_service(SetPOI,'setPOI', self.setPOI_callback)
    
    def getPOI_callback(self,request,response):
        script_directory = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_directory, 'poi.json')
        with open(file_path, 'r') as jsonFile:
            poi_data = json.load(jsonFile)
        
        for poi in poi_data['PointOfInterest']:
            msg = PointOfInterest()
            msg.point_id = poi['PID']
            msg.region_id = poi['RegionID']
            
            coordinates = Coordinate()
            coordinates.longitude = poi['Coordinates']['x']
            coordinates.latitude = poi['Coordinates']['y']
            msg.coordinates = coordinates

            msg.task = poi['Task']
            msg.status = poi['Status']
            
            response.poi.append(msg)
        return response

    def getROI_callback(self,request,response):
        script_directory = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_directory, 'roi.json')
        with open(file_path, 'r') as jsonFile:
            roi_data = json.load(jsonFile)
        
        for roi in roi_data['RegionOfInterest']:
            msg = RegionOfInterest()
            msg.region_id = roi['RegionID']

            bounds = []
            for bound in roi['Bounds']:
                coordinates = Coordinate()
                coordinates.longitude = bound['x']
                coordinates.latitude = bound['y']
                bounds.append(coordinates)
            msg.bounds = bounds

            rover_path = []
            for r_path in roi['RoverPath']:
                coordinates = Coordinate()
                coordinates.latitude = r_path['y']
                coordinates.longitude = r_path['x']
                rover_path.append(coordinates)
            msg.rover_path = rover_path

            uav_path = []
            for u_path in roi['UAVPath']:
                coordinates = Coordinate()
                coordinates.latitude = u_path['y']
                coordinates.longitude = u_path['x']
                uav_path.append(coordinates)
            msg.uav_path = uav_path

        response.roi.append(msg)
        return response
    
    def setPOI_callback(self,request,response):
        script_directory = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_directory, 'poi.json')
        with open(file_path, 'r') as jsonFile:
            poi_data = json.load(jsonFile)

        change_pid = request.point_id
        for poi in poi_data['PointOfInterest']:
            if change_pid == poi['PID']:
                poi['Status'] = request.status

        with open(file_path,'w') as jsonFile:
            json.dump(poi_data,jsonFile, indent =4)

        response.result = True
        return response

            
       
        

               
def main(args=None):
    rclpy.init(args=args)
    node = LanderServices()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
