#!/usr/bin/env python3
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

"""Rosbag retention against a real fault_manager_node on a SQLite file.

Each test starts the fault manager as its own process, feeds it sized payloads, confirms faults
over the ROS services and reads the outcome from the services, the database file and the bag
directory. A run ends with SIGTERM and an exit code of 0.
"""

import os
import pathlib
import shutil
import sys

from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import launch_testing.util

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from rosbag_fm_harness import (  # noqa: E402
    bag_dirs, MIB, poll, RetentionCase, sqlite_exec, Ticker,
)

RACE_REPETITIONS = 50
FUTURE_NS = 4_000_000_000 * 1_000_000_000


def generate_test_description():
    return LaunchDescription([launch_testing.util.KeepAliveProc(),
                              launch_testing.actions.ReadyToTest()]), {}


class TestRosbagQuota(RetentionCase):

    def test_quota_evicts_oldest_first(self):
        self.check_quota_evicts_oldest_first()

    def test_burst_shared_recording(self):
        """A recording shared by two faults counts once towards the quota and is evicted whole."""
        with self.fm_run(**{'snapshots.rosbag.max_bags_per_fault': 0,
                            'snapshots.rosbag.max_total_storage_mb': 7,
                            'snapshots.rosbag.duration_after_sec': 5.0}) as fm:
            burst = self.record(fm, 'BURST_A', 2, burst=('BURST_B',))
            self.assertEqual(bag_dirs(self.storage), [os.path.basename(burst)])
            self.assertEqual(sorted(self.stored_rows()),
                             [('BURST_A', burst), ('BURST_B', burst)])

            # Counted twice the burst plus two more recordings would be 8 MiB and over the quota.
            self.record(fm, 'FILL_1', 2)
            self.record(fm, 'FILL_2', 2)
            self.assert_recording_present('BURST_A', burst)
            self.assert_recording_present('BURST_B', burst)

            self.record(fm, 'FILL_3', 2)
            self.assert_recording_gone('BURST_A', burst)
            self.assert_recording_gone('BURST_B', burst)
            self.assertEqual([code for code, _ in self.stored_rows()],
                             ['FILL_1', 'FILL_2', 'FILL_3'])

    def test_oversized_recording_kept(self):
        """A recording larger than the quota evicts everything older and is kept itself."""
        with self.fm_run(**{'snapshots.rosbag.max_bags_per_fault': 0,
                            'snapshots.rosbag.max_total_storage_mb': 3}) as fm:
            older = {code: self.record(fm, code, 1) for code in ('OLD_A', 'OLD_B')}
            mark = fm.mark()
            big = self.record(fm, 'BIG', 4)
            self.assertIsNotNone(
                fm.wait_line(r'alone exceeds snapshots\.rosbag\.max_total_storage_mb \(3 MB\)',
                             since=mark),
                'no warning for the oversized recording:\n' + fm.output()[-3000:])
            for code, path in older.items():
                self.assert_recording_gone(code, path)
            self.assert_recording_present('BIG', big)

    def test_cap_and_quota_together(self):
        """The cap runs with the insert; the quota then sweeps what is left, oldest first."""
        params = {'snapshots.rosbag.max_bags_per_fault': 1,
                  'snapshots.rosbag.max_total_storage_mb': 5,
                  'snapshots.rosbag.auto_cleanup': False}

        with self.subTest('a: the cap frees the old recording before the quota counts'):
            self.new_scratch()
            with self.fm_run(**params) as fm:
                b = self.record(fm, 'CQ_B', 1)
                a_old = self.record(fm, 'CQ_A', 1.5)
                c = self.record(fm, 'CQ_C', 1.5)
                self.client.clear('CQ_A')
                a_new = self.record(fm, 'CQ_A', 1.5)
                self.assert_recording_gone('CQ_A', a_old)
                self.assert_recording_present('CQ_A', a_new)
                self.assert_recording_present('CQ_B', b)
                self.assert_recording_present('CQ_C', c)

        with self.subTest('b: a bigger replacement pushes the oldest other recording out'):
            self.new_scratch()
            with self.fm_run(**params) as fm:
                a_old = self.record(fm, 'CQ_A', 1.5)
                b = self.record(fm, 'CQ_B', 1.5)
                c = self.record(fm, 'CQ_C', 1.5)
                self.client.clear('CQ_A')
                a_new = self.record(fm, 'CQ_A', 3)
                self.assert_recording_gone('CQ_A', a_old)
                self.assert_recording_present('CQ_A', a_new)
                self.assert_recording_gone('CQ_B', b)
                self.assert_recording_present('CQ_C', c)

        with self.subTest('c: the cap leaves a burst a sibling holds, the quota evicts it whole'):
            self.new_scratch()
            with self.fm_run(**{**params, 'snapshots.rosbag.duration_after_sec': 3.0}) as fm:
                burst = self.record(fm, 'CQ_A', 1.5, burst=('CQ_X',))
                b = self.record(fm, 'CQ_B', 1.5)
                c = self.record(fm, 'CQ_C', 1.5)
                self.client.clear('CQ_A')
                a_new = self.record(fm, 'CQ_A', 1.5)
                self.assert_recording_gone('CQ_A', burst)
                self.assert_recording_gone('CQ_X', burst)
                self.assert_recording_present('CQ_A', a_new)
                self.assert_recording_present('CQ_B', b)
                self.assert_recording_present('CQ_C', c)

    def _seed_future_row(self, code):
        sqlite_exec(self.db, 'UPDATE rosbag_files SET created_at_ns = ? WHERE fault_code = ?',
                    FUTURE_NS, code)

    def test_new_recording_loses_the_cap(self):
        """The cap keeps the newest row by timestamp, so a new recording can lose to an old one."""
        params = {'snapshots.rosbag.max_bags_per_fault': 1, 'snapshots.rosbag.auto_cleanup': False}

        with self.subTest('plain'):
            self.new_scratch()
            with self.fm_run(**params) as fm:
                seeded = self.record(fm, 'LOSE_A', 1)
            self._seed_future_row('LOSE_A')
            with self.fm_run(**params) as fm:
                self.client.clear('LOSE_A')
                mark = fm.mark()
                self.client.publish_bytes(8, 128 * 1024)
                self.assertTrue(self.client.report('LOSE_A').accepted)
                done = fm.wait_line(r'Bag file (?:created|completed): (\S*fault_LOSE_A_\d+)',
                                    since=mark)
                self.assertIsNotNone(done, fm.output()[-3000:])
                fresh = done.group(1)
                self.assertNotEqual(fresh, seeded)
                self.assertTrue(poll(lambda: not os.path.exists(fresh)), f'{fresh} was kept')
                self.assertEqual(self.stored_rows(), [('LOSE_A', seeded)])
                self.assertEqual(bag_dirs(self.storage), [os.path.basename(seeded)])

        with self.subTest('burst'):
            self.new_scratch()
            with self.fm_run(**params) as fm:
                seeded = self.record(fm, 'LOSE_A', 1)
            self._seed_future_row('LOSE_A')
            with self.fm_run(**{**params, 'snapshots.rosbag.duration_after_sec': 3.0}) as fm:
                self.client.clear('LOSE_A')
                fresh = self.record(fm, 'LOSE_A', 1, burst=('LOSE_X',))
                self.assertNotEqual(fresh, seeded)
                self.assertEqual(sorted(self.stored_rows()),
                                 [('LOSE_A', seeded), ('LOSE_X', fresh)])
                self.assertEqual(sorted(bag_dirs(self.storage)),
                                 sorted([os.path.basename(seeded), os.path.basename(fresh)]))

    def _trigger(self, name, event, code=None):
        row = 'NEW' if event == 'INSERT' else 'OLD'
        condition = f" WHEN {row}.fault_code = '{code}'" if code else ''
        sqlite_exec(self.db,
                    f'CREATE TRIGGER {name} BEFORE {event} ON rosbag_files{condition} '
                    f"BEGIN SELECT RAISE(ABORT, 'pinned failure'); END")

    def _confirm_expecting_failure(self, fm, code, mib):
        mark = fm.mark()
        self.client.publish_bytes(int(mib * MIB) // (128 * 1024), 128 * 1024)
        self.assertTrue(self.client.report(code).accepted)
        failed = fm.wait_line(rf"Failed to store rosbag metadata for '(\S*fault_{code}_\d+)'",
                              since=mark)
        self.assertIsNotNone(failed, fm.output()[-3000:])
        stored = rf'Bag file (?:created|completed): \S*fault_{code}_\d+'
        self.assertEqual(fm.count_lines(stored, since=mark), 0)
        return failed.group(1)

    def test_store_failures(self):
        """A failing store discards the new bag; a failing quota sweep keeps it."""
        with self.subTest('i: insert fails, bag discarded, no post-roll'):
            self.new_scratch()
            with self.fm_run() as fm:
                self._trigger('fail_insert', 'INSERT')
                bag = self._confirm_expecting_failure(fm, 'STORE_A', 1)
                self.assertFalse(os.path.exists(bag))
                self.assertEqual(self.stored_rows(), [])
                self.assertEqual(bag_dirs(self.storage), [])

        with self.subTest('i: insert fails, bag discarded, post-roll'):
            self.new_scratch()
            with self.fm_run(**{'snapshots.rosbag.duration_after_sec': 1.0}) as fm:
                self._trigger('fail_insert', 'INSERT')
                bag = self._confirm_expecting_failure(fm, 'STORE_A', 1)
                self.assertFalse(os.path.exists(bag))
                self.assertEqual(self.stored_rows(), [])
                self.assertEqual(bag_dirs(self.storage), [])

        with self.subTest('ii: the cap cannot delete, the store fails, the new bag is discarded'):
            self.new_scratch()
            with self.fm_run(**{'snapshots.rosbag.auto_cleanup': False}) as fm:
                kept = self.record(fm, 'STORE_A', 1)
                self.client.clear('STORE_A')
                self._trigger('fail_delete', 'DELETE')
                bag = self._confirm_expecting_failure(fm, 'STORE_A', 1)
                self.assertNotEqual(bag, kept)
                self.assertFalse(os.path.exists(bag))
                self.assertEqual(self.stored_rows(), [('STORE_A', kept)])
                self.assertEqual(bag_dirs(self.storage), [os.path.basename(kept)])

        with self.subTest('iii: the quota sweep cannot delete, the new bag and its row are kept'):
            self.new_scratch()
            with self.fm_run(**{'snapshots.rosbag.max_bags_per_fault': 0,
                                'snapshots.rosbag.max_total_storage_mb': 3}) as fm:
                first = self.record(fm, 'STORE_A', 2)
                self._trigger('fail_delete', 'DELETE')
                mark = fm.mark()
                second = self.record(fm, 'STORE_B', 2)
                self.assertIsNotNone(
                    fm.wait_line(r'Failed to enforce the rosbag storage limit after storing '
                                 r"'\S*fault_STORE_B_\d+'", since=mark),
                    fm.output()[-3000:])
                self.assert_recording_present('STORE_B', second)
                self.assert_recording_present('STORE_A', first)

    def _race(self, **overrides):
        """Confirm and clear fault after fault; none may keep a row or a directory."""
        discarded = 0
        with self.fm_run(**overrides) as fm, Ticker(self.client):
            for i in range(RACE_REPETITIONS):
                code = f'RACE_{i}'
                mark = fm.mark()
                self.assertTrue(self.client.report(code).accepted)
                self.assertTrue(self.client.clear(code).success)
                outcome = fm.wait_line(
                    rf'Bag file (?:created|completed|discarded)[^\n]*?(\S*fault_{code}_\d+)'
                    rf"|No data buffered for fault '{code}'", since=mark)
                self.assertIsNotNone(outcome, f'no outcome for {code}:\n' + fm.output()[-3000:])
                if outcome.group(1) is None:
                    continue  # the ring buffer was empty at the confirm; nothing was recorded
                bag = outcome.group(1)
                if 'discarded' in outcome.group(0):
                    discarded += 1
                self.assertFalse(self.client.get_rosbag(code).success)
                self.assertEqual(self.row_paths(code), [])
                self.assertTrue(poll(lambda: not os.path.exists(bag)),
                                f'{bag} was kept for {code}')
        return discarded

    def test_clear_racing_store(self):
        """A fault cleared right after it confirms leaves neither a row nor a directory."""
        with self.subTest('inline store'):
            self.new_scratch()
            discarded = self._race(**{'snapshots.rosbag.duration_after_sec': 0.0})
            self.assertGreater(discarded, 0,
                               'no clear landed before the store, so the discard path never ran')
        with self.subTest('post-roll store'):
            self.new_scratch()
            self._race(**{'snapshots.rosbag.duration_after_sec': 0.3})

    def test_shutdown_during_post_roll(self):
        """SIGTERM inside the post-fault window keeps the recording, with its row."""
        with self.fm_run(**{'snapshots.rosbag.duration_after_sec': 30.0}) as fm:
            mark = fm.mark()
            self.client.publish_bytes(8, 128 * 1024)
            self.assertTrue(self.client.report('SHUT_A').accepted)
            self.assertIsNotNone(
                fm.wait_line(r'Recording [\d.]+s more after fault confirmation', since=mark))
            self.assertEqual(fm.terminate(), 0, fm.output()[-3000:])
        self.assertEqual(len(bag_dirs(self.storage)), 1)
        with self.fm_run(**{'snapshots.rosbag.duration_after_sec': 30.0}):
            served = self.client.get_rosbag('SHUT_A')
            self.assertTrue(served.success, served.error_message)
            self.assertTrue(os.path.isdir(served.file_path))
            self.assertEqual([code for code, _ in self.stored_rows()], ['SHUT_A'])

    def test_shutdown_right_after_clear(self):
        """SIGTERM as soon as ClearFault returns leaves no directory and a clean exit."""
        with self.fm_run() as fm:
            bag = self.record(fm, 'SHUT_B', 4)
            self.assertTrue(self.client.clear('SHUT_B').success)
            self.assertEqual(fm.terminate(), 0, fm.output()[-3000:])
        self.assertFalse(os.path.exists(bag))
        self.assertEqual(bag_dirs(self.storage), [])

    def test_stale_rows(self):
        """A row whose directory is gone is reaped on the next listing, with rosbag on or off."""
        with self.subTest('rosbag off on the next run'):
            self.new_scratch()
            with self.fm_run() as fm:
                bag = self.record(fm, 'STALE_A', 1)
            shutil.rmtree(bag)
            self.assertEqual(self.row_paths('STALE_A'), [bag])
            with self.fm_run(**{'snapshots.rosbag.enabled': False}):
                listed = self.client.list_rosbags()
                self.assertTrue(listed.success, listed.error_message)
                self.assertEqual(list(listed.fault_codes), [])
                self.assertEqual(self.stored_rows(), [])

        with self.subTest('rosbag on, directory removed while running'):
            self.new_scratch()
            with self.fm_run() as fm:
                bag = self.record(fm, 'STALE_A', 1)
                shutil.rmtree(bag)
                served = self.client.get_rosbag('STALE_A')
                self.assertFalse(served.success)
                self.assertIn('not found on disk', served.error_message)
                self.assertEqual(self.stored_rows(), [])
