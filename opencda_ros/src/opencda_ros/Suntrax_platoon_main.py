import uuid
import carla
import rclpy

from rclpy.node import Node

from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus, \
                           CarlaWorldInfo, CarlaStatus, CarlaEgoVehicleInfo, \
                           CarlaActorInfo

from .vehicle_data_subscriber import VehicleInfoSubscriber, CarlaDataSubscriber
from .coordinate_transform import geo_to_transform

def main(args=None):
    # read carla world 
    client = carla.Client('localhost', 2000)
    carla_world = client.get_world()
    carla_map = carla_world.get_map()
    geo_ref = carla_map.transform_to_geolocation(
            carla.Location(x=0, y=0, z=0))

    # note hard code for now, need to parse from object file
    vehicle_role_names = ['single_ADS_vehicle', 'mainline_ADS_vehicle_1']

    # init subscriber nodes
    rclpy.init(args=args)
    carla_data_subscriber = CarlaDataSubscriber(vehicle_role_names)

    # vehicle info subs
    vehicle_info_subscribers = []
    for vehicle_name in vehicle_role_names:
        vehicle_info_subscriber = VehicleInfoSubscriber(vehicle_name)
        vehicle_info_subscribers.append(vehicle_info_subscriber)

    # platoon manager 

    # vehicle manager

    # spectator 

    # simulation and ROS loop 
    try:
        while rclpy.ok():
            rclpy.spin_once(carla_data_subscriber, timeout_sec=0.01)
            for i,vehicle_info_subscriber in enumerate(vehicle_info_subscribers):
                rclpy.spin_once(vehicle_info_subscribers[i], timeout_sec=0.01)
            
            gnss_data_1 = vehicle_info_subscribers[0].get_gnss_data()
            gnss_data_2 = vehicle_info_subscribers[1].get_gnss_data()

            # transfer GPS to x,y,z location
            if gnss_data_1 is not None: 
                x, y, z = geo_to_transform(gnss_data_1.latitude,
                                          gnss_data_1.longitude,
                                          gnss_data_1.altitude,
                                          geo_ref.latitude,
                                          geo_ref.longitude, 0.0)
                location_1 = [x,y,z]
                print('------------------------')
                print("vehicle 1 location from GNSS: " + str(location_1))
            
            vehicle_translation = None
            vehicle_rotation = None

            #debug
            print('======== Debug stream ========')
            print("map: " + str(carla_data_subscriber.map))
            print("world_info: " + str(carla_data_subscriber.world_info))
            print("carla_status: " + str(carla_data_subscriber.carla_status))
            print("----- Vehicle data -----")
            print("vehicle GNSS data 1: " + str(gnss_data_1))
            print('------------------------')
            print("vehicle GNSS data 2: " + str(gnss_data_2))
            print('------------------------')
            print("==============================")
            print("")
            
            #
            # call OpenCDA module here: 
            #       (control = control_mnager.run_step(vehicle managers))
            #
            # rate.sleep()

    except KeyboardInterrupt:
        pass 

    carla_data_subscriber.destroy_node()
    for vehicle_info_subscriber in vehicle_info_subscribers:
        vehicle_info_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

