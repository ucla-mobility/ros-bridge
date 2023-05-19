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

# vehicle sensor subs
class VehicleInfoSubscriber(Node):
    def __init__(self, vehicle_role_name: str):
        super().__init__('vehicle_info_subscriber_' + vehicle_role_name)
        
        self.gnss_data = None
        self.imu_data = None

        self.gnss_subscription = self.create_subscription(
            NavSatFix,
            f'/carla/{vehicle_role_name}/gnss',
            self.gnss_callback,
            100
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            f'/carla/{vehicle_role_name}/imu',
            self.imu_callback,
            100
        )

    def gnss_callback(self, msg: NavSatFix):
        self.gnss_data = msg

    def imu_callback(self, msg: Imu):
        self.imu_data = msg

    def get_gnss_data(self):
        return self.gnss_data

    def get_imu_data(self):
        return self.imu_data


class CarlaDataSubscriber(Node):
    def __init__(self, vehicle_role_names):
        super().__init__('carla_data_subscriber')

        # vehicle role name to subscribe to corresponding topic
        self.vehicle_role_names = vehicle_role_names
        self.ADS_vehicle_data = {}

        # simulation info
        self.map = None
        self.world_info = None
        self.carla_status = None

        # tf for all sensors
        self.tf_meta = TFMessage()

        # creat vehicle info subs
        # self.vehicle_info_subscriber = VehicleInfoSubscriber(self.vehicle_role_names[0])

        # ROS subscription world
        self.map_subscription = self.create_subscription(
            String,
            '/carla/map',
            self.carla_map_callback,
            10
        )

        self.carla_status_subscription = self.create_subscription(
            CarlaStatus,
            '/carla/status',
            self.carla_status_callback,
            10
        )

        self.world_info_subscription = self.create_subscription(
            CarlaWorldInfo,
            '/carla/world_info',
            self.world_info_callback,
            10
        )

        self.TF_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            100
        )

    def carla_map_callback(self, msg: String):
        self.map = msg
        # self.get_logger().info(f'The subscribed map info is: {msg}')

    def carla_status_callback(self, msg: CarlaStatus):
        self.carla_status = msg
        # self.get_logger().info(f'The subscribed CARLA status is: {msg}')

    def world_info_callback(self, msg: CarlaWorldInfo):
        self.world_info = msg
        # self.get_logger().info(f'The subscribed World Info is: {msg}')

    def tf_callback(self, msg: TFMessage):
        self.tf_meta = msg
        # self.get_logger().info(f'The subscribed tf Info is: {msg}')


    # consider use another node to subscribe vehicle info one by one

    # def vehicle_gnss_callback(self, msg: String)
    #     self.vehicle_gnss = msg
    #     self.get_logger().info(f'Vehicle GNSS: {msg}')

    # def vehicle_imu_callback(self, msg: String)
    #     self.vehicle_imu = msg
    #     self.get_logger().info(f'Vehicle imu: {msg}')

    # information getter 
    # def get_carla_map_data(self):
    #     return self.map

    # def get_carla_status_data(self):
    #     return self.carla_status

    # def get_world_info_data(self):
    #     return self.world_info

    #
    # init vehicle manager here based on role_name
    #

    #
    # init openCDA managers here
    #

# Node main functions
def main(args=None):
    rclpy.init(args=args)

    vehicle_role_names = ['single_ADS_vehicle', 'mainline_ADS_vehicle_1']      
    carla_data_subscriber = CarlaDataSubscriber(vehicle_role_names)
    # init openCDA moduels here

    # rate = VehicleDataSubscriber.create_rate(10)  # 10 Hz
    rate =  carla_data_subscriber.create_rate(10)

    try:
        while rclpy.ok():
            rclpy.spin_once(carla_data_subscriber, timeout_sec=0.1)
            rclpy.spin_once(carla_data_subscriber.vehicle_info_subscriber, timeout_sec=0.1)

            gnss_data = carla_data_subscriber.vehicle_info_subscriber.get_gnss_data()
            imu_data = carla_data_subscriber.vehicle_info_subscriber.get_imu_data()

            vehicle_translation = None
            vehicle_rotation = None

            # if gnss_data is not None:
                # Do something with the GNSS data
                # carla_data_subscriber.get_logger().info(f'GNSS Data: {gnss_data}')

            # if imu_data is not None:
                # Do something with the GNSS data
                # carla_data_subscriber.get_logger().info(f'IMU Data: {imu_data}')

            
            # parse tf info
            for transform in carla_data_subscriber.tf_meta.transforms:
                # print(transform.child_frame_id)
                if transform.child_frame_id == carla_data_subscriber.vehicle_role_names[0] +'/gnss':
                    vehicle_translation = transform.transform.translation
                    vehicle_rotation = transform.transform.rotation

            #debug
            print('======== Debug stream ========')
            print("map: " + str(carla_data_subscriber.map))
            print("world_info: " + str(carla_data_subscriber.world_info))
            print("carla_status: " + str(carla_data_subscriber.carla_status))
            print("----- Vehicle data -----")
            print("vehicle GNSS data: " + str(gnss_data))
            print("==============================")
            print("")
            
            #
            # call OpenCDA module here: 
            #       (control = control_mnager.run_step(vehicle managers))
            #
            # rate.sleep()

    except KeyboardInterrupt:
        pass 

    #
    # Add publisher here:
    #       publish control
    #

    carla_data_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


