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

"""
Rosbag retention against a real fault_manager_node on a real PostgreSQL server.

ROS2_MEDKIT_TEST_PG_CONN names the server, as `postgresql://user:password@host:port/database`.
Each test creates its own database on it. An unreachable server fails the test.
"""

import os
import pathlib
import sys
import urllib.parse
import uuid

from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import launch_testing.util
import psycopg2

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from rosbag_fm_harness import bag_dirs, RetentionCase  # noqa: E402


def generate_test_description():
    return LaunchDescription([launch_testing.util.KeepAliveProc(),
                              launch_testing.actions.ReadyToTest()]), {}


class TestRosbagQuotaPostgres(RetentionCase):

    def setUp(self):
        super().setUp()
        base = os.environ.get('ROS2_MEDKIT_TEST_PG_CONN')
        self.assertTrue(base, 'ROS2_MEDKIT_TEST_PG_CONN is not set')
        self.admin_url = base
        self.database = 'rosbag_pin_' + uuid.uuid4().hex[:12]
        self._admin('CREATE DATABASE ' + self.database)
        self.addCleanup(self._admin, f'DROP DATABASE IF EXISTS {self.database} WITH (FORCE)')
        parts = urllib.parse.urlsplit(base)
        self.url = urllib.parse.urlunsplit(parts._replace(path='/' + self.database))

    def _admin(self, statement):
        conn = psycopg2.connect(self.admin_url)
        try:
            conn.autocommit = True
            with conn.cursor() as cur:
                cur.execute(statement)
        finally:
            conn.close()

    def storage_params(self):
        return {'storage_type': 'postgres', 'database_url': self.url}

    def stored_rows(self):
        conn = psycopg2.connect(self.url)
        try:
            with conn.cursor() as cur:
                cur.execute('SELECT fault_code, file_path FROM rosbag_files ORDER BY id')
                return cur.fetchall()
        finally:
            conn.close()

    def test_quota_evicts_oldest_first(self):
        self.check_quota_evicts_oldest_first()

    def test_clear_removes_the_row_and_the_directory(self):
        with self.fm_run() as fm:
            bag = self.record(fm, 'PG_A', 1)
            self.assert_recording_present('PG_A', bag)
            self.assertTrue(self.client.clear('PG_A').success)
            self.assert_recording_gone('PG_A', bag)
            self.assertEqual(self.stored_rows(), [])
            self.assertEqual(bag_dirs(self.storage), [])
