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

from lerobot_so101_example.camera_monitor import FrozenCheck


def test_counts_identical_frames_in_a_row():
    check = FrozenCheck()
    assert check.update(b'a') == 1
    assert check.update(b'a') == 2
    assert check.update(b'a') == 3


def test_a_new_frame_resets_the_count():
    check = FrozenCheck()
    for _ in range(5):
        check.update(b'a')
    assert check.update(b'b') == 1
    assert check.update(b'b') == 2
