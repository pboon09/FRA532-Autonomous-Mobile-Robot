import math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState, Imu
from tf_transformations import euler_from_quaternion


class BagReader:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.joint_states = []
        self.imu_data = []

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

        return self.joint_states, self.imu_data

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

    def get_synchronized_data(self):
        if not self.joint_states or not self.imu_data:
            self.read()

        joint_times = [js['timestamp'] for js in self.joint_states]
        imu_times = [imu['timestamp'] for imu in self.imu_data]

        synced_data = []
        imu_idx = 0

        for js in self.joint_states:
            t = js['timestamp']

            while imu_idx < len(self.imu_data) - 1 and self.imu_data[imu_idx + 1]['timestamp'] <= t:
                imu_idx += 1

            if imu_idx < len(self.imu_data):
                imu = self.imu_data[imu_idx]

                synced_data.append({
                    'timestamp': t,
                    'rad_l': js['rad_l'],
                    'rad_r': js['rad_r'],
                    'imu_yaw': imu['yaw']
                })

        return synced_data

    def reset(self):
        self.joint_states = []
        self.imu_data = []
