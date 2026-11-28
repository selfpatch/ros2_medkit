# Copyright 2026 ahmedsleem109
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

"""Episode runner: runs a campaign of attempts and reports failed ones.

For each attempt it resets the policy and the simulation, waits until the
simulation reports the episode done, and records the outcome. Attempts are
appended to a JSON-lines file and the last ones are published on `~/attempts`.

A failed attempt is reported to the ros2_medkit fault manager as
EPISODE_FAILED, which records a rosbag of the attempt. EPISODE_FAILED is
reported PASSED at the start of every attempt, so each failed attempt is its
own occurrence with its own recording. An aborted attempt (the robot or the
policy did not answer) is not a policy failure and is not reported as one.
Start and end are wall-clock times, the clock the fault manager stamps faults
with.
"""

import json
import os
import threading
import time

from lerobot_so101_example.fault_reporting import FaultReporter
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

EPISODE_FAILED = 'EPISODE_FAILED'


class EpisodeRunner(Node):

    def __init__(self):
        super().__init__('episode_runner')
        self.declare_parameter('num_episodes', 10)
        self.declare_parameter('autostart', False)
        self.declare_parameter('gap_sec', 1.0)
        self.declare_parameter('episode_timeout_sec', 300.0)
        # No simulation step for this long: the attempt is aborted (the policy died, say)
        self.declare_parameter('stall_timeout_sec', 20.0)
        self.declare_parameter('robot_ns', '/so101/so101_sim')
        self.declare_parameter('policy_reset_service', '/lerobot_policy/reset')
        self.declare_parameter('checkpoint', '')
        self.declare_parameter('results_file', '/data/results/attempts.jsonl')
        self.declare_parameter('history_size', 50)

        ns = self.get_parameter('robot_ns').value
        self._calls = ReentrantCallbackGroup()
        self._sim_reset = self.create_client(Trigger, f'{ns}/reset', callback_group=self._calls)
        self._policy_reset = self.create_client(
            Trigger, self.get_parameter('policy_reset_service').value, callback_group=self._calls
        )
        self.create_subscription(
            String, f'{ns}/task_status', self._on_status, 10, callback_group=self._calls
        )
        self._faults = FaultReporter(self, callback_group=self._calls)
        self._attempts_pub = self.create_publisher(String, '~/attempts', 10)
        self.create_service(
            Trigger,
            '~/start_campaign',
            self._on_start_campaign,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self._done = threading.Event()
        self._last_status = None
        self._last_status_time = time.monotonic()
        self._attempts = []
        self._thread = None
        self._next_number = 1
        # The simulation counts episodes from 0 after a restart: ids carry the run start.
        self._run_id = time.strftime('%Y%m%d-%H%M%S', time.gmtime())
        if self.get_parameter('autostart').value:
            self._start(int(self.get_parameter('num_episodes').value))

    def _on_start_campaign(self, request, response):
        num = int(self.get_parameter('num_episodes').value)
        response.success = self._start(num)
        response.message = (
            f'campaign of {num} attempts started'
            if response.success
            else 'a campaign is already running'
        )
        return response

    def _start(self, num):
        if self._thread is not None and self._thread.is_alive():
            return False
        self._thread = threading.Thread(target=self._run_campaign, args=(num,), daemon=True)
        self._thread.start()
        return True

    def _on_status(self, msg):
        status = json.loads(msg.data)
        self._last_status = status
        self._last_status_time = time.monotonic()
        if status.get('done'):
            self._done.set()

    def _call(self, client, name):
        if not client.wait_for_service(timeout_sec=60.0):
            raise RuntimeError(f'{name} is not available')
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 60.0
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError(f'{name} did not answer')
            time.sleep(0.01)
        result = future.result()
        if not result.success:
            raise RuntimeError(f'{name} failed: {result.message}')
        return result.message

    def _run_campaign(self, num):
        gap = float(self.get_parameter('gap_sec').value)
        self.get_logger().info(f'campaign of {num} attempts started')
        successes = 0
        for _ in range(num):
            attempt = self._run_attempt()
            successes += attempt['outcome'] == 'success'
            self._record(attempt)
            time.sleep(gap)
        self.get_logger().info(f'campaign done: {successes}/{num} succeeded')

    def _run_attempt(self):
        number = self._next_number
        self._next_number += 1
        attempt = {
            'number': number,
            'episode_id': None,
            'seed': None,
            'checkpoint': self.get_parameter('checkpoint').value,
            'start': time.time(),
            'end': None,
            'steps': 0,
            'max_lift_m': 0.0,
            'outcome': None,
            'abort_reason': None,
        }
        self._done.clear()
        self._last_status = None
        self._faults.passed(EPISODE_FAILED)
        try:
            self._call(self._policy_reset, 'policy reset')
            info = json.loads(self._call(self._sim_reset, 'simulation reset'))
            attempt['episode_id'] = f"{self._run_id}-ep{info['episode']:05d}"
            attempt['seed'] = info['seed']
            reason = self._wait_done()
            status = self._last_status
            if status is not None:
                attempt['steps'] = status['step']
                attempt['max_lift_m'] = status['max_lift_m']
            if reason is not None:
                attempt['outcome'], attempt['abort_reason'] = 'aborted', reason
            else:
                attempt['outcome'] = 'success' if status['success'] else 'failure'
        except RuntimeError as error:
            attempt['outcome'], attempt['abort_reason'] = 'aborted', str(error)
        attempt['end'] = time.time()
        if attempt['outcome'] == 'failure':
            fields = ('episode_id', 'number', 'seed', 'start', 'end', 'checkpoint')
            self._faults.failed(
                EPISODE_FAILED, 'error', json.dumps({k: attempt[k] for k in fields})
            )
        summary = (
            f"attempt {number} ({attempt['episode_id']}): {attempt['outcome']} after "
            f"{attempt['steps']} steps, {attempt['end'] - attempt['start']:.1f} s"
        )
        # One call site per severity: rclpy refuses to change it between calls.
        if attempt['outcome'] == 'aborted':
            self.get_logger().warning(f"{summary}: {attempt['abort_reason']}")
        else:
            self.get_logger().info(summary)
        return attempt

    def _wait_done(self):
        """Return None once the simulation reports done, else why the attempt is given up."""
        timeout = float(self.get_parameter('episode_timeout_sec').value)
        stall = float(self.get_parameter('stall_timeout_sec').value)
        deadline = time.monotonic() + timeout
        self._last_status_time = time.monotonic()
        while not self._done.wait(0.5):
            now = time.monotonic()
            if now > deadline:
                return f'timed out after {timeout:.0f} s'
            if stall > 0 and now - self._last_status_time > stall:
                return f'no simulation step for {stall:.0f} s (did the policy stop?)'
        return None

    def _record(self, attempt):
        self._attempts.append(attempt)
        del self._attempts[: -int(self.get_parameter('history_size').value)]
        self._attempts_pub.publish(String(data=json.dumps(self._attempts)))
        path = self.get_parameter('results_file').value
        if path:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'a') as results:
                results.write(json.dumps(attempt) + '\n')


def main():
    rclpy.init()
    node = EpisodeRunner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
