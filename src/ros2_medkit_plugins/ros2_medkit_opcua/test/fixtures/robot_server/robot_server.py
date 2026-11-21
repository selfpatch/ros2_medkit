#!/usr/bin/env python3
# Copyright 2026 mfaferek93
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

"""
asyncua server shaped like a robot controller, for browse_path tests.

The address space uses the namespace URIs and BrowseNames of OPC UA DI and
OPC 40010 Robotics, built by hand (no nodeset files):

  Objects
    di:DeviceSet
      cell:Robot1
        rob:SafetyStates / cell:SafetyState / di:ParameterSet / rob:EmergencyStop
        di:ParameterSet / rob:SpeedOverride          (writable)
    line:Spares
      line:Spare                                     (Boolean)
      line:SpareSetpoint                             (writable)

EmergencyStop and Spare have the same numeric identifier in their namespaces,
and so have SpeedOverride and SpareSetpoint. ``--layout first`` registers the
namespaces as DI, Robotics, cell, line and ``--layout swapped`` as cell, line,
DI, Robotics, so a restart with the other layout makes the NodeId a client
held for a robot variable name the spare one. ``--no-robot`` leaves DeviceSet
empty.

Commands on stdin, one per line, each answered on stdout:
  set <estop|spare|speed|setpoint> <value>   ->  OK <name>=<value>
  get <estop|spare|speed|setpoint>           ->  VALUE <name>=<value>
  slow <seconds>                             ->  OK slow=<seconds>
``slow`` delays every Browse answer. EOF on stdin stops the server.

Usage: robot_server.py --port <port> [--layout first|swapped] [--no-robot]
                       [--estop 0|1] [--slow <seconds>]
"""

import argparse
import asyncio
import logging
import sys

from asyncua import Server, ua
from asyncua.server import internal_session

DI = 'http://opcfoundation.org/UA/DI/'
ROBOTICS = 'http://opcfoundation.org/UA/Robotics/'
CELL = 'urn:medkit:test:cell'
LINE = 'urn:medkit:test:line'

browse_delay = 0.0


async def _slow_browse(self, params):
    if browse_delay > 0:
        await asyncio.sleep(browse_delay)
    return self.iserver.view_service.browse(params)


internal_session.InternalSession.browse = _slow_browse


async def build(server, args):
    """Create the address space and return the variables by command name."""
    order = [DI, ROBOTICS, CELL, LINE] if args.layout == 'first' else [CELL, LINE, DI, ROBOTICS]
    ns = {uri: await server.register_namespace(uri) for uri in order}
    di, rob, cell, line = ns[DI], ns[ROBOTICS], ns[CELL], ns[LINE]

    async def add_object(parent, namespace, identifier, name_ns, name):
        return await parent.add_object(ua.NodeId(identifier, namespace),
                                       ua.QualifiedName(name, name_ns))

    async def add_variable(parent, namespace, identifier, name, value, variant_type):
        node = await parent.add_variable(ua.NodeId(identifier, namespace),
                                         ua.QualifiedName(name, namespace), value, variant_type)
        return node

    variables = {}
    device_set = await add_object(server.nodes.objects, di, 5001, di, 'DeviceSet')
    if not args.no_robot:
        robot = await add_object(device_set, cell, 5100, cell, 'Robot1')
        states = await add_object(robot, rob, 5200, rob, 'SafetyStates')
        state = await add_object(states, cell, 5201, cell, 'SafetyState')
        state_params = await add_object(state, di, 5202, di, 'ParameterSet')
        variables['estop'] = await add_variable(
            state_params, rob, 6001, 'EmergencyStop', bool(args.estop), ua.VariantType.Boolean)
        robot_params = await add_object(robot, di, 5300, di, 'ParameterSet')
        variables['speed'] = await add_variable(
            robot_params, rob, 6002, 'SpeedOverride', 100.0, ua.VariantType.Float)
        await variables['speed'].set_writable()
    spares = await add_object(server.nodes.objects, line, 5400, line, 'Spares')
    variables['spare'] = await add_variable(
        spares, line, 6001, 'Spare', False, ua.VariantType.Boolean)
    variables['setpoint'] = await add_variable(
        spares, line, 6002, 'SpareSetpoint', 0.0, ua.VariantType.Float)
    await variables['setpoint'].set_writable()
    return variables


async def run_command(variables, line):
    global browse_delay
    parts = line.split()
    if len(parts) == 2 and parts[0] == 'slow':
        browse_delay = float(parts[1])
        return f'OK slow={parts[1]}'
    if len(parts) < 2 or parts[1] not in variables:
        return f'ERR {line}'
    node = variables[parts[1]]
    if parts[0] == 'get':
        return f'VALUE {parts[1]}={await node.read_value()}'
    if parts[0] == 'set' and len(parts) == 3:
        variant_type = await node.read_data_type_as_variant_type()
        value = parts[2] == '1' if variant_type == ua.VariantType.Boolean else float(parts[2])
        await node.write_value(ua.Variant(value, variant_type))
        return f'OK {parts[1]}={parts[2]}'
    return f'ERR {line}'


async def main():
    global browse_delay
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--layout', choices=['first', 'swapped'], default='first')
    parser.add_argument('--no-robot', action='store_true')
    parser.add_argument('--estop', type=int, default=0)
    parser.add_argument('--slow', type=float, default=0.0)
    args = parser.parse_args()
    browse_delay = args.slow
    logging.basicConfig(level=logging.ERROR)

    server = Server()
    await server.init()
    server.set_endpoint(f'opc.tcp://127.0.0.1:{args.port}')
    server.set_server_name('medkit robot test server')
    variables = await build(server, args)
    loop = asyncio.get_running_loop()
    async with server:
        print(f'READY port={args.port} layout={args.layout}', flush=True)
        while True:
            line = await loop.run_in_executor(None, sys.stdin.readline)
            if not line:
                break
            if line.strip():
                print(await run_command(variables, line.strip()), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(asyncio.run(main()))
