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

"""The safe band and the allocator that draws from it.

These are the properties that do not need a process to prove: which domains the
band contains, that the selector never leaves it, that a lock port already held
by somebody else is stepped over rather than fatal, and that a full band is
reported as such instead of quietly becoming domain 0.
"""

import contextlib
import socket

import medkit_domain
import pytest

EXPECTED_BAND = tuple(range(1, 101)) + tuple(range(215, 232))


class ReservedBand:
    """A handful of domains this process has proved it can hold, and holds.

    The lock port of a domain is ``32768 + domain``, which is the floor of the
    kernel's default ephemeral range: any process on the machine can be handed
    one of those ports at any moment. That is the whole reason the allocator
    skips a domain whose port it cannot bind, and a test helper that assumed it
    could bind all 117 at once was making exactly the assumption the band
    reasoning rejects. So a test picks its band by binding, not by naming: what
    it gets is whatever it actually holds.

    ``release(n)`` hands domains back so the allocator can take them, which is
    how a test arranges a known number of free slots.
    """

    def __init__(self, domains, sockets):
        self.domains = list(domains)
        self._sockets = list(sockets)

    @property
    def spec(self):
        """Format the band the way MEDKIT_TEST_DOMAIN_BAND wants it."""
        return ','.join(str(domain) for domain in self.domains)

    def release(self, count=None):
        """Give back *count* domains, or all of them, and return which."""
        count = len(self._sockets) if count is None else count
        freed = []
        for _ in range(count):
            if not self._sockets:
                break
            sock = self._sockets.pop()
            freed.append(self.domains[len(self._sockets)])
            sock.close()
        return freed

    def close(self):
        for sock in self._sockets:
            sock.close()
        self._sockets = []


@contextlib.contextmanager
def reserve_band(count, candidates=None):
    """Hold *count* domains chosen because their lock port could be bound.

    Never assumes a particular domain is free. Fails only if the machine cannot
    spare *count* of the whole band, which is a broken machine rather than a
    test that got unlucky.
    """
    candidates = medkit_domain.USABLE_DOMAINS if candidates is None else candidates
    domains, sockets = [], []
    for domain in candidates:
        if len(domains) == count:
            break
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.bind(('', medkit_domain.PORT_BASE + domain))
        except OSError:
            sock.close()
            continue
        domains.append(domain)
        sockets.append(sock)
    reserved = ReservedBand(domains, sockets)
    try:
        assert len(domains) == count, (
            f'could only hold {len(domains)} of {count} domains out of '
            f'{len(candidates)} candidates; this machine has almost no free lock ports'
        )
        yield reserved
    finally:
        reserved.close()


def test_band_is_the_documented_range():
    assert medkit_domain.USABLE_DOMAINS == EXPECTED_BAND
    assert len(medkit_domain.USABLE_DOMAINS) == 117


def test_band_endpoints_are_included():
    for endpoint in (1, 100, 215, 231):
        assert endpoint in medkit_domain.USABLE_DOMAINS


def test_band_excludes_the_domains_that_are_not_safe():
    # 0 is the developer default, 101-214 overlap the kernel ephemeral range,
    # 232 runs past the top of the UDP port space.
    for unsafe in (0, 101, 150, 214, 232, 233):
        assert unsafe not in medkit_domain.USABLE_DOMAINS


def test_selector_stays_inside_the_band_and_covers_all_of_it():
    selector = medkit_domain.BandSelector(medkit_domain.USABLE_DOMAINS, start=0)
    drawn = [selector() for _ in range(len(EXPECTED_BAND) * 2)]
    assert set(drawn) <= set(EXPECTED_BAND)
    assert set(drawn) == set(EXPECTED_BAND)


def test_selector_starts_somewhere_other_than_the_first_domain_sometimes():
    starts = set()
    for _ in range(50):
        starts.add(medkit_domain.BandSelector(medkit_domain.USABLE_DOMAINS)())
    assert len(starts) > 1, 'every allocator starting at the same domain maximises contention'


@pytest.mark.parametrize('endpoint', [1, 100, 215, 231])
def test_a_band_of_one_endpoint_yields_that_endpoint_or_refuses(endpoint):
    # Whether this endpoint is free right now is not ours to decide: its lock
    # port sits in the range the kernel hands out. Both outcomes are asserted,
    # so the test never assumes and never skips. What it pins is that the
    # selector cannot leave the band it was given.
    try:
        with medkit_domain.hold_domains(1, domains=(endpoint,), wait_timeout=0.5) as domains:
            assert domains == [endpoint]
    except medkit_domain.NoDomainAvailable:
        pass


def test_a_lock_port_held_by_an_outsider_is_skipped_not_fatal():
    with reserve_band(3) as band:
        blocked = band.domains[0]
        band.release(2)  # the other two go back; only `blocked` stays held
        with medkit_domain.hold_domains(2, domains=band.domains) as domains:
            assert blocked not in domains
            assert sorted(domains) == sorted(band.domains[1:])


def test_a_band_with_every_port_held_is_reported_not_silently_downgraded():
    reached_body = False
    with reserve_band(3) as band:
        with pytest.raises(medkit_domain.NoDomainAvailable) as excinfo:
            with medkit_domain.hold_domains(1, domains=band.domains, wait_timeout=0.5):
                reached_body = True
        message = str(excinfo.value)
    assert not reached_body, 'a full band must not hand out a domain'
    assert 'ROS_DOMAIN_ID' in message
    assert '3' in message, 'the message must say how large the band it exhausted is'


def test_concurrent_holders_never_share_a_domain():
    with reserve_band(2) as band:
        band.release()
        with medkit_domain.hold_domains(1, domains=band.domains) as first:
            with medkit_domain.hold_domains(1, domains=band.domains) as second:
                assert first[0] != second[0]


def test_domains_are_released_when_the_holder_exits():
    with reserve_band(1) as band:
        only = band.release()[0]
        with medkit_domain.hold_domains(1, domains=(only,)) as domains:
            assert domains == [only]
        with medkit_domain.hold_domains(1, domains=(only,)) as domains:
            assert domains == [only]


def test_a_failure_inside_the_body_still_releases_the_domain():
    # domain_coordinator.domain_id() swallows OSError around its own yield, so an
    # OSError escaping the body must not be able to confuse the release path.
    with reserve_band(1) as band:
        only = band.release()[0]
        with pytest.raises(OSError):
            with medkit_domain.hold_domains(1, domains=(only,)):
                raise OSError('the test under the allocator blew up')
        with medkit_domain.hold_domains(1, domains=(only,)) as domains:
            assert domains == [only]


def test_a_band_override_can_narrow_the_band_but_never_widen_it():
    assert medkit_domain.parse_band('1-4,215') == (1, 2, 3, 4, 215)
    assert medkit_domain.default_band({medkit_domain.BAND_ENV: '7,8'}) == (7, 8)
    assert medkit_domain.default_band({}) == medkit_domain.USABLE_DOMAINS
    for unsafe in ('0', '101', '232', '1-101'):
        with pytest.raises(ValueError) as excinfo:
            medkit_domain.parse_band(unsafe)
        assert 'safe band excludes' in str(excinfo.value)


def test_environment_export_names_the_primary_and_the_extras():
    env = {'ROS_DOMAIN_ID': '0', 'MEDKIT_SECONDARY_DOMAINS': '229'}
    medkit_domain.apply_to_environ([7, 8, 9], env)
    assert env['ROS_DOMAIN_ID'] == '7'
    assert env['MEDKIT_SECONDARY_DOMAINS'] == '8,9'

    medkit_domain.apply_to_environ([7], env)
    assert env['ROS_DOMAIN_ID'] == '7'
    assert 'MEDKIT_SECONDARY_DOMAINS' not in env
