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

"""Runtime ``ROS_DOMAIN_ID`` allocation for tests.

A test asks for one or more domains when it starts and holds them, through an
open socket, for exactly as long as it runs. The kernel closes that socket when
the holder dies for any reason, including SIGKILL, so a domain is never leaked
by a crashed run.

The lock is ``domain_coordinator.domain_id`` from ``ament_cmake_ros``, which is
installed on every distro we build for. It binds a TCP socket on
``32768 + domain`` and holds it for the lifetime of its context. That socket is
what makes the allocation global to the machine rather than to one CTest run:
two ``ctest`` processes started by two different ``colcon`` package jobs contend
for the same ports, which no CTest ``RESOURCE_LOCK`` can do.

Two things here are ours rather than upstream's:

* the band, see ``USABLE_DOMAINS`` below;
* waiting. ``domain_id`` raises as soon as it has tried its selector 100 times,
  which under ``ctest -j`` simply means "everything is busy right now". A test
  that cannot have a domain yet has to wait for one, with a bound, not fail.

---------------------------------------------------------------------------
Why the band is not simply 1-232
---------------------------------------------------------------------------

RTPS gives every DDS domain a 250-port slice of the UDP space:

  slice(d) = [7400 + 250 * d, 7400 + 250 * d + 249]

Cyclone DDS 0.10.5, what both Humble and Jazzy ship, binds the multicast
sockets at offsets 0 and 1 unconditionally, on every configuration -
ddsi_portmapping.c forces the participant index to 0 for both multicast cases,
so the index never enters into it. Those sockets do set both SO_REUSEPORT and
SO_REUSEADDR, but a collision there is still fatal: Linux only grants the reuse
exemption when every socket that ever held the port opted in, so an ordinary
socket already sitting on it blocks the bind regardless. A failed multicast bind
aborts domain creation, so this is the failure that actually kills a node:

  ddsi_udp_create_conn: failed to bind to ANY:53150: address in use
  rmw_create_node: failed to create domain, error Error

53150 is 7400 + 250 * 183, the multicast base port of domain 183 - the failure
above is exactly the case this band protects against.

The unicast sockets, at offsets 10 + 2p and 11 + 2p for participant index p,
depend on Discovery/ParticipantIndex. On Humble, rmw_cyclonedds_cpp emits no
<Discovery> element, so the default "none" applies and the single unicast socket
gets a kernel-assigned port with no relation to the domain. On Jazzy,
rmw_cyclonedds_cpp injects <ParticipantIndex>auto</ParticipantIndex> with
MaxAutoParticipantIndex 32, so unicast does land on offsets 10 through 75 - but
a collision there is not fatal, it just advances to the next participant index.

Fast DDS 3.6.2 (Lyrical) derives every listening port from the formula and never
falls back to a kernel-assigned one; its participant id mutates by +2 for up to
100 tries on collision. A failed multicast bind there is only a warning, so
discovery degrades instead of the process dying.

What keeping a domain's slice out of the ephemeral range buys, then: on every
distro it protects offsets 0 and 1, the one collision that is fatal. On Jazzy it
additionally covers offsets 10 through 75. On Humble the unicast socket sits
outside the scheme entirely and needs no protection, since a kernel-assigned
port is free by construction.

The protection is deterministic rather than statistical: once a socket with the
reuse options holds a port, the kernel will not also hand that port out as an
ephemeral one. The race runs one way - the unrelated process has to grab the
port before the participant starts, never the other way around.

The kernel hands out ephemeral ports from net.ipv4.ip_local_port_range, which
defaults to 32768-60999. Any unrelated process - a browser, a package manager, a
second test run - can be given a port in that range, and if it is one of ours
the whole domain stops working. Mapped back through the formula, the default
range covers domains 101 to 214: domain 101's slice, 32650-32899, already
overlaps the range starting at 32768.

The usable domains are the ones whose whole slice sits outside that range: 1-100
and 215-231. Domain 0 stays free because it is the ROS 2 default a developer
shell uses, and 232 is dropped because its slice runs past 65535. Upstream's
``domain_coordinator`` selector uses 1-100 only; the extra 17 are ours and are
the reason a selector is passed in at all.
"""

import contextlib
import os
import random
import time

import domain_coordinator

#: Where this file and its sibling scripts live, for callers that need to spawn
#: one of them.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

#: Every DDS domain a test may use. See the module docstring for the derivation.
USABLE_DOMAINS = tuple(range(1, 101)) + tuple(range(215, 232))

#: The coordination port of a domain is ``PORT_BASE + domain``. Fixed by
#: ``domain_coordinator``; mirrored here so tests can reason about the lock.
PORT_BASE = domain_coordinator.impl.PORT_BASE

#: How long to keep trying before giving up, in seconds. Under ``ctest -j`` on a
#: large machine more tests can be in flight than the band has domains, and the
#: right answer for the excess is to wait for one to be handed back.
DEFAULT_WAIT_TIMEOUT = float(os.environ.get('MEDKIT_TEST_DOMAIN_WAIT', '180'))

#: Environment variable a test carries to say how many domains it needs.
DOMAIN_COUNT_ENV = 'MEDKIT_TEST_DOMAINS'

#: Narrows the band this process draws from, as a comma separated list of
#: domains and inclusive ranges, for example '1-8,215'. Validated to be a subset
#: of USABLE_DOMAINS, so it can pin the allocator to part of the band but can
#: never point it at a domain the band excludes. The allocator's own tests use
#: it to work against a handful of domains they have proved they can bind,
#: rather than assuming the whole band is theirs to take.
BAND_ENV = 'MEDKIT_TEST_DOMAIN_BAND'

#: CTest label a test carries to say it creates no ROS entities at all, and so
#: needs no domain. A label rather than an environment entry on purpose: the
#: environment is inherited by everything a test starts, and a test that spawns
#: a wrapped child would hand it its own declaration.
NO_DOMAIN_LABEL = 'no_ros_domain'

#: Where the extra domains of a multi-domain test are published.
SECONDARY_ENV = 'MEDKIT_SECONDARY_DOMAINS'

_RETRY_INTERVAL = 0.05


class NoDomainAvailable(RuntimeError):
    """Raised when the whole band stayed busy for longer than the caller waited."""


def parse_band(spec):
    """Turn '1-8,215' into a tuple of domains, refusing anything outside the band."""
    domains = []
    for part in spec.split(','):
        part = part.strip()
        if not part:
            continue
        if '-' in part:
            low, _, high = part.partition('-')
            try:
                bounds = range(int(low), int(high) + 1)
            except ValueError:
                raise ValueError(f'{BAND_ENV}: {part!r} is not a range of domains')
            domains.extend(bounds)
        else:
            try:
                domains.append(int(part))
            except ValueError:
                raise ValueError(f'{BAND_ENV}: {part!r} is not a domain')
    if not domains:
        raise ValueError(f'{BAND_ENV}={spec!r} names no domains')
    outside = sorted(set(domains) - set(USABLE_DOMAINS))
    if outside:
        raise ValueError(
            f'{BAND_ENV}={spec!r} names {outside}, which the safe band excludes. It can '
            'narrow the band, never widen it - see the module docstring for why those '
            'domains are not usable.'
        )
    return tuple(dict.fromkeys(domains))


def default_band(env=None):
    """Return the band this process draws from."""
    if env is None:
        env = os.environ
    spec = env.get(BAND_ENV)
    return parse_band(spec) if spec else USABLE_DOMAINS


class BandSelector:
    """Walk the band in order, from a random starting point.

    ``domain_coordinator.domain_id`` calls this for each bind attempt, so the
    only contract is "return the next candidate". Starting at a random offset
    matters under load: a fixed start means every process on the machine tries
    the same domain first and walks the same order, so the first free slot is
    contended by all of them at once.
    """

    def __init__(self, domains=USABLE_DOMAINS, start=None):
        if not domains:
            raise ValueError('BandSelector needs at least one domain to draw from')
        self._domains = tuple(domains)
        if start is None:
            start = random.randrange(len(self._domains))
        self._next = start % len(self._domains)

    def __call__(self):
        domain = self._domains[self._next]
        self._next = (self._next + 1) % len(self._domains)
        return domain


def _acquire_one(selector, domains, deadline, announce):
    """Hold one domain, waiting for the band to free up if it has to.

    Returns the entered context manager, so the caller can release it later.
    """
    waited_from = None
    while True:
        manager = domain_coordinator.domain_id(selector)
        try:
            return manager, manager.__enter__()
        except RuntimeError:
            pass
        if waited_from is None:
            waited_from = time.monotonic()
            announce(
                f'all {len(domains)} DDS domains are in use, waiting for one to be '
                f'released (up to {max(0.0, deadline - waited_from):.0f}s)'
            )
        if time.monotonic() >= deadline:
            raise NoDomainAvailable(
                f'no free ROS_DOMAIN_ID after waiting '
                f'{time.monotonic() - waited_from:.0f}s: every one of the '
                f'{len(domains)} domains in the safe band is held by another test. '
                'Nothing may run without a domain, so this test was not started - '
                'running on the default domain 0 would make it see every node on the '
                'machine. Raise MEDKIT_TEST_DOMAIN_WAIT if the machine is simply slow, '
                'or lower the test parallelism.'
            )
        time.sleep(_RETRY_INTERVAL)


@contextlib.contextmanager
def hold_domains(count=1, domains=None, wait_timeout=None, announce=None):
    """Hold *count* distinct DDS domains for the duration of the context.

    The domains are released when the context exits, and by the kernel closing
    the sockets if the process dies instead.
    """
    if domains is None:
        domains = default_band()
    if count < 1:
        raise ValueError(f'a test needs at least one domain, asked for {count}')
    if wait_timeout is None:
        wait_timeout = DEFAULT_WAIT_TIMEOUT
    if announce is None:
        def announce(message):
            print(f'[medkit-domain] {message}', flush=True)

    selector = BandSelector(domains)
    deadline = time.monotonic() + wait_timeout
    managers = []
    acquired = []
    try:
        for _ in range(count):
            manager, domain = _acquire_one(selector, domains, deadline, announce)
            managers.append(manager)
            acquired.append(domain)
        yield acquired
    finally:
        # Always a clean exit, never the caller's exception: domain_id() catches
        # OSError around its own yield, so throwing one into it would send it
        # back around its retry loop instead of releasing the socket.
        for manager in reversed(managers):
            manager.__exit__(None, None, None)


def apply_to_environ(domains, env=None):
    """Publish the held domains the way every test reads them."""
    if env is None:
        env = os.environ
    env['ROS_DOMAIN_ID'] = str(domains[0])
    if len(domains) > 1:
        env[SECONDARY_ENV] = ','.join(str(domain) for domain in domains[1:])
    else:
        env.pop(SECONDARY_ENV, None)
    # The count is an instruction to this wrapper, not to anything it starts.
    # Left in place, a test that runs a wrapped command of its own would inherit
    # its parent's request and ask for that many domains again.
    env.pop(DOMAIN_COUNT_ENV, None)


def requested_domain_count(env=None):
    """Return how many domains the caller's environment asks for."""
    if env is None:
        env = os.environ
    raw = env.get(DOMAIN_COUNT_ENV, '1')
    try:
        count = int(raw)
    except ValueError:
        raise SystemExit(
            f'{DOMAIN_COUNT_ENV}={raw!r} is not a number. It is set by the CMake '
            'helpers in ROS2MedkitTestDomain.cmake and must be a count of domains.'
        )
    if count < 1:
        raise SystemExit(
            f'{DOMAIN_COUNT_ENV}={count} reached a test that goes through the domain '
            'wrapper. A test needs at least one domain, or it must not be wrapped at '
            'all - medkit_test_needs_no_domain() is how a test says it needs none.'
        )
    return count
