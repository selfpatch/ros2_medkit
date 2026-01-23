#!/usr/bin/env python3
# Copyright 2025 mfaferek93
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Integration tests for rosbag capture feature (Issue #120)."""

import os
import tempfile
import time
import unittest

import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from launch import LaunchDescription
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, GetRosbag, ReportFault


def get_coverage_env():
    """Get environment variables for gcov coverage data collection."""
    try:
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix('ros2_medkit_fault_manager')
        workspace = os.path.dirname(os.path.dirname(pkg_prefix))
        build_dir = os.path.join(workspace, 'build', 'ros2_medkit_fault_manager')

        if os.path.exists(build_dir):
            return {
                'GCOV_PREFIX': build_dir,
                'GCOV_PREFIX_STRIP': str(build_dir.count(os.sep)),
            }
    except Exception:
        pass
    return {}


# Create a temp directory for rosbag storage that persists for test duration
ROSBAG_STORAGE_PATH = tempfile.mkdtemp(prefix='rosbag_test_')


def generate_test_description():
    """Generate launch description with fault_manager node with rosbag enabled.

    Launch order:
    1. Start background publisher node (Python script)
    2. After delay, start fault_manager_node (publishers are now registered)
    3. Signal ready for tests
    """
    # Use Python script for publishers to ensure proper topic type registration
    # ros2 topic pub has issues with topic type discovery in some configurations
    publisher_script = '''
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Temperature
from std_msgs.msg import String
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        import os
        domain_id = os.environ.get('ROS_DOMAIN_ID', 'not set')
        self.get_logger().info(f'Using ROS_DOMAIN_ID: {domain_id}')

        # Use sensor data QoS to match rosbag capture subscriptions
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.temp_pub = self.create_publisher(Temperature, '/test/temperature', qos)
        self.string_pub = self.create_publisher(String, '/test/status', qos)
        self.timer = self.create_timer(0.1, self.publish)
        self.counter = 0
        self.get_logger().info('TestPublisher started - publishing to /test/temperature and /test/status')

    def publish(self):
        temp_msg = Temperature()
        temp_msg.temperature = 25.0 + self.counter * 0.1
        temp_msg.variance = 0.1
        self.temp_pub.publish(temp_msg)

        string_msg = String()
        string_msg.data = f'status_{self.counter}'
        self.string_pub.publish(string_msg)
        self.counter += 1
        if self.counter % 50 == 0:
            self.get_logger().info(f'Published {self.counter} messages')
            # Log available topics and types for debugging
            topics = self.get_topic_names_and_types()
            topic_info = [(t, types) for t, types in topics if 'test' in t]
            if topic_info:
                self.get_logger().info(f'Test topics visible: {topic_info}')

def main():
    rclpy.init()
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

    # Write publisher script to temp file and run it
    import tempfile
    script_file = tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False)
    script_file.write(publisher_script)
    script_file.close()

    # Use explicit ROS settings to ensure processes can discover each other
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '42'
    env['ROS_LOCALHOST_ONLY'] = '1'  # Force localhost-only discovery for faster performance

    test_publisher = launch.actions.ExecuteProcess(
        cmd=['python3', script_file.name],
        output='screen',
        name='test_publisher',
        env=env,
    )

    fault_manager_env = get_coverage_env()
    fault_manager_env['ROS_DOMAIN_ID'] = '42'
    fault_manager_env['ROS_LOCALHOST_ONLY'] = '1'

    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        additional_env=fault_manager_env,
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -1,  # Single report confirms immediately
            # Rosbag configuration
            'snapshots.rosbag.enabled': True,
            'snapshots.rosbag.duration_sec': 2.0,  # 2 second buffer
            'snapshots.rosbag.duration_after_sec': 0.5,  # 0.5 second after confirm
            # Use explicit topics - faster than discovery and more reliable for testing
            'snapshots.rosbag.topics': '/test/temperature,/test/status',
            'snapshots.rosbag.format': 'sqlite3',
            'snapshots.rosbag.storage_path': ROSBAG_STORAGE_PATH,
            'snapshots.rosbag.max_bag_size_mb': 10,
            'snapshots.rosbag.max_total_storage_mb': 50,
            'snapshots.rosbag.auto_cleanup': True,
            # lazy_start=false: Start recording immediately
            'snapshots.rosbag.lazy_start': False,
        }],
    )

    # Delay fault_manager start so test_publisher has time to register topics
    # DDS discovery can take several seconds to propagate topic types
    delayed_fault_manager = launch.actions.TimerAction(
        period=8.0,
        actions=[fault_manager_node],
    )

    return (
        LaunchDescription([
            # Start publisher node first
            test_publisher,
            # Start fault_manager after delay
            delayed_fault_manager,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'fault_manager_node': fault_manager_node,
            'test_publisher': test_publisher,
        },
    )


class TestRosbagCaptureIntegration(unittest.TestCase):
    """Integration tests for rosbag capture feature.

    Background publishers (/test/temperature, /test/status) are provided by the
    launch description and run continuously throughout the tests. These publishers
    are started BEFORE the fault_manager_node, ensuring topics exist when rosbag
    capture initializes.
    """

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context and create service clients."""
        # Use same ROS settings as launch processes to ensure discovery
        os.environ['ROS_DOMAIN_ID'] = '42'
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        cls.node = Node('test_rosbag_client')

        # Create service clients
        cls.report_fault_client = cls.node.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        cls.clear_fault_client = cls.node.create_client(
            ClearFault, '/fault_manager/clear_fault'
        )
        cls.get_rosbag_client = cls.node.create_client(
            GetRosbag, '/fault_manager/get_rosbag'
        )

        # Wait for services to be available
        assert cls.report_fault_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'
        assert cls.clear_fault_client.wait_for_service(timeout_sec=15.0), \
            'clear_fault service not available'
        assert cls.get_rosbag_client.wait_for_service(timeout_sec=15.0), \
            'get_rosbag service not available'

        # Give time for rosbag capture to fill buffer with messages from
        # background publishers (duration_sec=2.0 configured)
        time.sleep(3.0)

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 context."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call_service(self, client, request, timeout_sec=10.0):
        """Call a service and wait for response."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def _report_fault(self, fault_code, description='Test fault'):
        """Report a fault and return the response."""
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = description
        request.source_id = '/test_node'
        return self._call_service(self.report_fault_client, request)

    def test_01_rosbag_created_on_fault_confirmation(self):
        """
        Test that rosbag file is created when fault is confirmed.

        Background publishers are continuously publishing, and the ring buffer
        should already have messages from setUpClass wait time.

        Verifies:
        - Messages are being recorded in ring buffer
        - Fault confirmation triggers bag file creation
        - GetRosbag service returns valid file info
        """
        fault_code = 'ROSBAG_TEST_001'

        # Report fault - buffer should already have messages from background publishers
        response = self._report_fault(fault_code, 'Rosbag integration test fault')
        self.assertTrue(response.accepted)
        print(f'Fault {fault_code} reported and confirmed')

        # Wait for post-fault recording to complete
        time.sleep(1.0)  # duration_after_sec=0.5 + buffer

        # Query rosbag info
        rosbag_request = GetRosbag.Request()
        rosbag_request.fault_code = fault_code

        rosbag_response = self._call_service(self.get_rosbag_client, rosbag_request)

        self.assertTrue(rosbag_response.success,
                        f'GetRosbag failed: {rosbag_response.error_message}')
        self.assertGreater(len(rosbag_response.file_path), 0)
        self.assertEqual(rosbag_response.format, 'sqlite3')
        self.assertGreater(rosbag_response.duration_sec, 0)
        self.assertGreater(rosbag_response.size_bytes, 0)

        # Verify file exists
        self.assertTrue(os.path.exists(rosbag_response.file_path),
                        f'Rosbag file not found: {rosbag_response.file_path}')

        print(f'Rosbag created: {rosbag_response.file_path}')
        print(f'  Duration: {rosbag_response.duration_sec:.2f}s')
        print(f'  Size: {rosbag_response.size_bytes} bytes')
        print(f'  Format: {rosbag_response.format}')

    def test_02_rosbag_auto_cleanup_on_clear(self):
        """
        Test that rosbag file is deleted when fault is cleared (auto_cleanup=true).

        Verifies:
        - Rosbag exists after fault confirmation
        - Clearing fault deletes the rosbag file
        - GetRosbag returns error after clear
        """
        fault_code = 'ROSBAG_CLEANUP_TEST'

        # Report fault - buffer has messages from background publishers
        response = self._report_fault(fault_code, 'Cleanup test fault')
        self.assertTrue(response.accepted)

        # Wait for post-fault recording
        time.sleep(1.0)

        # Verify rosbag exists
        rosbag_request = GetRosbag.Request()
        rosbag_request.fault_code = fault_code
        rosbag_response = self._call_service(self.get_rosbag_client, rosbag_request)

        self.assertTrue(rosbag_response.success)
        bag_path = rosbag_response.file_path
        self.assertTrue(os.path.exists(bag_path))
        print(f'Rosbag exists before clear: {bag_path}')

        # Clear the fault
        clear_request = ClearFault.Request()
        clear_request.fault_code = fault_code

        clear_response = self._call_service(self.clear_fault_client, clear_request)
        self.assertTrue(clear_response.success)
        print('Fault cleared')

        # Small delay for cleanup
        time.sleep(0.5)

        # Verify rosbag is deleted
        self.assertFalse(os.path.exists(bag_path),
                         f'Rosbag should be deleted but still exists: {bag_path}')
        print('Rosbag auto-deleted on clear')

        # GetRosbag should now fail
        rosbag_response2 = self._call_service(self.get_rosbag_client, rosbag_request)
        self.assertFalse(rosbag_response2.success)
        print(f'GetRosbag after clear: {rosbag_response2.error_message}')

    def test_03_get_rosbag_nonexistent_fault(self):
        """Test GetRosbag returns error for nonexistent fault."""
        request = GetRosbag.Request()
        request.fault_code = 'NONEXISTENT_ROSBAG_FAULT'

        response = self._call_service(self.get_rosbag_client, request)

        self.assertFalse(response.success)
        self.assertIn('not found', response.error_message.lower())
        print(f'Nonexistent fault error: {response.error_message}')

    def test_04_get_rosbag_empty_fault_code(self):
        """Test GetRosbag returns error for empty fault code."""
        request = GetRosbag.Request()
        request.fault_code = ''

        response = self._call_service(self.get_rosbag_client, request)

        self.assertFalse(response.success)
        self.assertIn('empty', response.error_message.lower())
        print(f'Empty fault code error: {response.error_message}')

    def test_05_multiple_faults_separate_bags(self):
        """
        Test that multiple faults create separate rosbag files.

        Verifies:
        - Each confirmed fault has its own rosbag
        - Rosbags are independent
        """
        fault_codes = ['MULTI_BAG_A', 'MULTI_BAG_B']
        bag_paths = []

        # Report multiple faults - buffer has messages from background publishers
        for fault_code in fault_codes:
            response = self._report_fault(fault_code, f'Multi-bag test: {fault_code}')
            self.assertTrue(response.accepted)
            time.sleep(1.0)  # Wait for post-fault recording

        # Verify each fault has its own rosbag
        for fault_code in fault_codes:
            rosbag_request = GetRosbag.Request()
            rosbag_request.fault_code = fault_code

            rosbag_response = self._call_service(self.get_rosbag_client, rosbag_request)

            self.assertTrue(rosbag_response.success,
                            f'GetRosbag failed for {fault_code}: {rosbag_response.error_message}')
            self.assertTrue(os.path.exists(rosbag_response.file_path))
            bag_paths.append(rosbag_response.file_path)
            print(f'Rosbag for {fault_code}: {rosbag_response.file_path}')

        # Verify bags are different files
        self.assertNotEqual(bag_paths[0], bag_paths[1])
        print('Multiple faults have separate rosbag files')

    def test_06_rosbag_contains_recorded_duration(self):
        """
        Test that rosbag duration approximately matches configuration.

        duration_sec=2.0 (ring buffer) + duration_after_sec=0.5 = ~2.5s expected
        """
        fault_code = 'DURATION_TEST'

        # Report fault - buffer should have ~duration_sec worth of messages
        response = self._report_fault(fault_code, 'Duration test fault')
        self.assertTrue(response.accepted)

        # Wait for post-fault recording (duration_after_sec=0.5)
        time.sleep(1.0)

        # Get rosbag info
        rosbag_request = GetRosbag.Request()
        rosbag_request.fault_code = fault_code

        rosbag_response = self._call_service(self.get_rosbag_client, rosbag_request)

        self.assertTrue(rosbag_response.success)

        # Duration should be approximately duration_sec + duration_after_sec
        # Allow some tolerance for timing
        expected_min = 1.5  # At least some data
        expected_max = 4.0  # Upper bound with tolerance
        self.assertGreaterEqual(rosbag_response.duration_sec, expected_min,
                                f'Duration too short: {rosbag_response.duration_sec}')
        self.assertLessEqual(rosbag_response.duration_sec, expected_max,
                             f'Duration too long: {rosbag_response.duration_sec}')

        print(f'Rosbag duration: {rosbag_response.duration_sec:.2f}s '
              f'(expected: ~2.5s with tolerance)')


@launch_testing.post_shutdown_test()
class TestRosbagCaptureShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    def test_exit_code(self, proc_info):
        """Verify fault_manager exits cleanly.

        Only check fault_manager_node exit code - the test_publisher Python script
        exits with SIGINT which has a non-zero exit code.
        """
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process='fault_manager_node'
        )

    def test_cleanup_temp_directory(self):
        """Clean up temporary rosbag storage directory."""
        import shutil
        if os.path.exists(ROSBAG_STORAGE_PATH):
            shutil.rmtree(ROSBAG_STORAGE_PATH, ignore_errors=True)
            print(f'Cleaned up temp directory: {ROSBAG_STORAGE_PATH}')
