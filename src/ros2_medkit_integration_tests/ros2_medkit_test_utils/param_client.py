# Copyright 2026 bburda
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

"""Read and write a node's parameters over its parameter services.

One rclpy node serves every call, so DDS discovery happens once and not once
per `ros2 param` process. The services are the ones `ros2 param` uses.
"""

import time

from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import (
    GetParameters,
    ListParameters,
    SetParameters,
    SetParametersAtomically,
)
import rclpy
from rclpy.parameter import Parameter

NOT_SET = ParameterType.PARAMETER_NOT_SET


class ParameterClient:
    """Parameter-service client for any node on the domain."""

    def __init__(self, name='param_probe', timeout=30.0):
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        self._node = rclpy.create_node(name, start_parameter_services=False)
        self._timeout = timeout

    def close(self):
        """Destroy the node, and the context if this client created it."""
        self._node.destroy_node()
        if self._owns_context:
            rclpy.shutdown()

    def _call(self, srv_type, node, suffix, request):
        """Call `<node>/<suffix>`, retrying until the service answers.

        A fresh client per attempt: a gateway that restarted under the same
        name is matched anew, and one that is still starting is waited for.
        """
        deadline = time.monotonic() + self._timeout
        while True:
            client = self._node.create_client(srv_type, f'{node}/{suffix}')
            try:
                if client.wait_for_service(timeout_sec=2.0):
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
                    if future.done():
                        return future.result()
            finally:
                self._node.destroy_client(client)
            if time.monotonic() >= deadline:
                raise TimeoutError(f'{node}/{suffix} did not answer within {self._timeout}s')

    def list_names(self, node):
        """Return every parameter name of `node`."""
        request = ListParameters.Request()
        request.depth = ListParameters.Request.DEPTH_RECURSIVE
        return list(self._call(ListParameters, node, 'list_parameters', request).result.names)

    def get_message(self, node, name):
        """Return the ParameterValue message; its type is NOT_SET for an undeclared name."""
        request = GetParameters.Request()
        request.names = [name]
        values = self._call(GetParameters, node, 'get_parameters', request).values
        return values[0] if values else ParameterValue()

    def is_not_set(self, node, name):
        """Return True when the node reports the parameter's type as PARAMETER_NOT_SET."""
        return self.get_message(node, name).type == NOT_SET

    def get(self, node, name):
        """Return the value as Python data; raise if the parameter is not set."""
        message = self.get_message(node, name)
        if message.type == NOT_SET:
            raise LookupError(f'{node} has no parameter {name}')
        parameter = Parameter.from_parameter_msg(ParameterMsg(name=name, value=message))
        return parameter.value

    def set_many(self, node, values):
        """Set `{name: value}` in one non-atomic request; return one result per entry."""
        request = SetParameters.Request()
        request.parameters = [Parameter(n, value=v).to_parameter_msg() for n, v in values.items()]
        return list(self._call(SetParameters, node, 'set_parameters', request).results)

    def set_one(self, node, name, value):
        """Set one parameter; return its SetParametersResult."""
        return self.set_many(node, {name: value})[0]

    def set_atomically(self, node, values):
        """Set `{name: value}` as one atomic batch; return its SetParametersResult."""
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(n, value=v).to_parameter_msg() for n, v in values.items()]
        response = self._call(SetParametersAtomically, node, 'set_parameters_atomically', request)
        return response.result
