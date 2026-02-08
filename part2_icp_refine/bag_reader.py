import math
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState, Imu, LaserScan
from tf_transformations import euler_from_quaternion


class BagReader:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.joint_states = []
        self.imu_data = []
        self.scan_data = []

    def read(self):
        storage_options = StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {t.name: t.type for t in topic_types}

        while reader.has_next():
            topic, data, timestamp = reader.read_next()

            if topic == '/joint_states':
                msg = deserialize_message(data, JointState)
                joint_data = self._parse_joint_state(msg, timestamp)
                if joint_data:
                    self.joint_states.append(joint_data)

            elif topic == '/imu':
                msg = deserialize_message(data, Imu)
                imu_data = self._parse_imu(msg, timestamp)
                if imu_data:
                    self.imu_data.append(imu_data)

            elif topic == '/scan':
                msg = deserialize_message(data, LaserScan)
                scan_data = self._parse_scan(msg, timestamp)
                if scan_data:
                    self.scan_data.append(scan_data)

        return self.joint_states, self.imu_data, self.scan_data

    def _parse_joint_state(self, msg, timestamp):
        try:
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')

            return {
                'timestamp': timestamp * 1e-9,
                'rad_l': msg.position[left_idx],
                'rad_r': msg.position[right_idx]
            }
        except (ValueError, IndexError):
            return None

    def _parse_imu(self, msg, timestamp):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        return {
            'timestamp': timestamp * 1e-9,
            'yaw': yaw
        }

    def _parse_scan(self, msg, timestamp):
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment/2, msg.angle_increment)

        if len(angles) > len(ranges):
            angles = angles[:len(ranges)]
        elif len(ranges) > len(angles):
            ranges = ranges[:len(angles)]

        valid_indices = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        return {
            'timestamp': timestamp * 1e-9,
            'ranges': ranges,
            'angles': angles,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_min': msg.range_min,
            'range_max': msg.range_max
        }

    def get_synchronized_data(self):
        if not self.joint_states or not self.imu_data or not self.scan_data:
            self.read()

        joint_times = [js['timestamp'] for js in self.joint_states]
        imu_times = [imu['timestamp'] for imu in self.imu_data]
        scan_times = [scan['timestamp'] for scan in self.scan_data]

        synced_data = []
        imu_idx = 0
        scan_idx = 0

        for js in self.joint_states:
            t = js['timestamp']

            while imu_idx < len(self.imu_data) - 1 and self.imu_data[imu_idx + 1]['timestamp'] <= t:
                imu_idx += 1

            while scan_idx < len(self.scan_data) - 1 and self.scan_data[scan_idx + 1]['timestamp'] <= t:
                scan_idx += 1

            if imu_idx < len(self.imu_data) and scan_idx < len(self.scan_data):
                imu = self.imu_data[imu_idx]
                scan = self.scan_data[scan_idx]

                synced_data.append({
                    'timestamp': t,
                    'rad_l': js['rad_l'],
                    'rad_r': js['rad_r'],
                    'imu_yaw': imu['yaw'],
                    'scan_ranges': scan['ranges'],
                    'scan_angles': scan['angles']
                })

        return synced_data

    def reset(self):
        self.joint_states = []
        self.imu_data = []
        self.scan_data = []
