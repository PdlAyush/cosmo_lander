#!/usr/bin/env python3

from lander.srv import  GetPOI, GetROI, SetPOI
from lander.msg import Coordinate, PointOfInterest, RegionOfInterest

import rclpy
from rclpy.node import Node


class LanderSubscriber(Node):

    def __init__(self):
        super().__init__('lander')
        self.poi_subscription = self.create_subscription(PointOfInterest, 'lander/pois', self.poi_callerback,10)
        self.roi_subscription = self.create_subscription(RegionOfInterest, 'lander/rois', self.roi_callerback,10)
    
    def poi_callerback(self, msg ):

        self.get_logger().info('Received PointOfInterest message:')
        self.get_logger().info('PID: %d' % msg.point_id)
        self.get_logger().info('RegionID: %d' % msg.region_id)
        self.get_logger().info('Coordinates: x=%f, y=%f' % (msg.coordinates.longitude, msg.coordinates.latitude))
        self.get_logger().info('Task: %s' % msg.task)
        self.get_logger().info('Status: %s' % msg.status)
    
    def roi_callerback(self, msg):
        self.get_logger().info('Received RegionOfInterest message:')
        self.get_logger().info('RegionID: %d' % msg.region_id)

        self.get_logger().info('Bounds:')
        for bound in msg.bounds:
            self.get_logger().info('Coordinates: x=%f, y=%f' % (bound.latitude, bound.longitude))

        self.get_logger().info('RoverPath:')
        for rpath in msg.rover_path:
            self.get_logger().info('Coordinates: x=%f, y=%f' % (rpath.latitude, rpath.longitude))

        self.get_logger().info('UAVPath:')
        for upath in msg.uav_path:
            self.get_logger().info('Coordinates: x=%f, y=%f' % (upath.latitude, upath.longitude))

        




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = LanderSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
