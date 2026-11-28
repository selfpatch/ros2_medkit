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

"""Watches a LeRobot policy from inside, by wrapping three of its methods.

Every LeRobot policy (`PreTrainedPolicy`) has them, and LeRobot's loops
(`lerobot-rollout`'s inference engines, `lerobot-record`, `lerobot-eval`,
`predict_action`) call `reset` and `select_action` or `predict_action_chunk`.
In lerobot 0.7, 23 policies run their model from `select_action` through
`predict_action_chunk`; SmolVLA, gaussian_actor, lingbot_va and longwam do
not, so for them only the `select_action` checks work (one warning says so).

- `reset()`: a new episode.
- `select_action(batch)`: one control step. Timed (slow_inference, measure
  'step'); its camera frames are checked (frozen_camera).
- `predict_action_chunk(batch)`: one model inference, producing a chunk of
  future actions. Timed (measure 'chunk'); when called from `select_action`,
  the chunk is published as introspection with the step it was made at.

Loops that call `predict_action_chunk` without `select_action` (lerobot's
chunk and RTC engines) execute the chunk elsewhere, so the monitor cannot
tell the step: their chunks are not published (one warning). Their frames
and inference time are still checked. Those engines call the policy from
their own thread; the monitor's state is behind a lock.

The wrappers call the original method first and never change what it
returns. Each check runs on its own: an error inside one is logged once and
reaches neither the other checks nor the policy's caller.
"""

import functools
import logging
import re
import sys
import threading
import time

from lerobot_medkit.checks import FrozenFrames, SlowInference

IMAGE_PREFIX = 'observation.images.'
logger = logging.getLogger('lerobot_medkit')


def _camera_name(key):
    return re.sub(r'[^A-Z0-9]+', '_', key[len(IMAGE_PREFIX):].upper()).strip('_')


def _frames_equal(a, b):
    if a.shape != b.shape:
        return False
    try:
        import torch

        if isinstance(a, torch.Tensor):
            return bool(torch.equal(a, b))
    except ImportError:
        pass
    import numpy as np

    return bool(np.array_equal(a, b))


def _actions(output):
    """Return the action tensor of a `Tensor | PolicyOutput`."""
    if isinstance(output, dict):
        return output.get('action')
    return output


def _to_numpy(chunk):
    if hasattr(chunk, 'detach'):
        chunk = chunk.detach().float().cpu().numpy()
    import numpy as np

    chunk = np.asarray(chunk, dtype=np.float32)
    if chunk.ndim == 3:  # (batch, steps, dim): the first environment
        chunk = chunk[0]
    return chunk


class PolicyMonitor:
    """Monitors `policy` with `config` (a MedkitConfig), reporting to `transport`.

    `postprocess(chunk)` turns the raw chunk from `predict_action_chunk` (model
    units, usually normalized) into the units the introspection should carry,
    for example the policy's postprocessor. Without it the raw chunk is sent.
    """

    def __init__(
        self,
        policy,
        config,
        transport,
        postprocess=None,
        clock=time.monotonic,
        wall_clock=time.time,
    ):
        self._policy = policy
        self._config = config
        self._transport = transport
        self._postprocess = postprocess
        self._clock = clock
        self._wall_clock = wall_clock
        self._slow = SlowInference(
            config.slow_inference.max_ms,
            config.slow_inference.window,
            config.slow_inference.clear_ratio,
        )
        self._cameras = {}  # image key -> FrozenFrames, or None when its code collides
        self._camera_codes = {}  # fault code -> image key
        self._originals = {}
        self._lock = threading.RLock()
        self._local = threading.local()  # in_step, per thread
        self._failures = {}  # where -> count
        self._warned_direct = False
        # select_action calls, until one reaches predict_action_chunk
        self._steps_without_chunk = 0
        self._chunk_in_step_seen = False
        self._warned_no_chunk = False
        self._logs = []  # (level, message, exc_info), logged once the lock is released
        self.episode = 0
        self.step = 0

    @property
    def attached(self):
        return bool(self._originals)

    def attach(self):
        if self.attached:
            raise RuntimeError('already attached')
        self._warn_on_window()
        for name, wrapper in (
            ('reset', self._wrap_reset),
            ('select_action', self._wrap_select_action),
            ('predict_action_chunk', self._wrap_predict_action_chunk),
        ):
            original = getattr(self._policy, name, None)
            if original is None:
                continue
            # An instance attribute (torch.compile puts one on predict_action_chunk)
            # is put back on detach; a method from the class is uncovered again.
            own = vars(self._policy).get(name)
            self._originals[name] = own
            setattr(self._policy, name, wrapper(original))
        return self

    def detach(self):
        for name, own in self._originals.items():
            if own is None:
                delattr(self._policy, name)
            else:
                setattr(self._policy, name, own)
        self._originals.clear()

    def close(self):
        self.detach()
        if self._failures:
            logger.warning('lerobot_medkit: monitor failures: %s', self._failures)
        self._transport.close()

    def _warn_on_window(self):
        slow = self._config.slow_inference
        steps = getattr(getattr(self._policy, 'config', None), 'n_action_steps', None)
        if (
            slow.max_ms > 0
            and slow.measure == 'step'
            and isinstance(steps, int)
            and slow.window < steps
        ):
            logger.warning(
                'lerobot_medkit: slow_inference.window %d is shorter than n_action_steps %d: '
                'a slow model makes the fault come and go with each inference. Use '
                'measure: chunk, or a window of at least %d',
                slow.window,
                steps,
                steps,
            )

    # Wrappers

    def _wrap_reset(self, original):
        @functools.wraps(original)
        def reset(*args, **kwargs):
            result = original(*args, **kwargs)
            with self._lock:
                self._guard('reset', self._on_reset)
            self._flush_logs()
            return result

        return reset

    def _wrap_select_action(self, original):
        @functools.wraps(original)
        def select_action(batch, *args, **kwargs):
            self._local.in_step = True
            self._local.chunk_in_step = False
            start = self._clock()
            try:
                result = original(batch, *args, **kwargs)
            finally:
                self._local.in_step = False
            elapsed_ms = (self._clock() - start) * 1000.0
            with self._lock:
                self._guard('frozen_camera', self._check_frames, batch)
                if self._config.slow_inference.measure == 'step':
                    self._guard('slow_inference', self._check_slow, elapsed_ms)
                self._note_chunk_in_step(self._local.chunk_in_step)
                self.step += 1
            self._flush_logs()
            return result

        return select_action

    def _wrap_predict_action_chunk(self, original):
        @functools.wraps(original)
        def predict_action_chunk(batch, *args, **kwargs):
            stamp = self._wall_clock()
            start = self._clock()
            result = original(batch, *args, **kwargs)
            elapsed_ms = (self._clock() - start) * 1000.0
            in_step = getattr(self._local, 'in_step', False)
            if in_step:
                self._local.chunk_in_step = True
            with self._lock:
                if not in_step:
                    self._guard('frozen_camera', self._check_frames, batch)
                if self._config.slow_inference.measure == 'chunk':
                    self._guard('slow_inference', self._check_slow, elapsed_ms)
                if self._config.introspection.enabled:
                    if in_step:
                        self._guard('introspection', self._publish, result, elapsed_ms, stamp)
                    elif not self._warned_direct:
                        self._warned_direct = True
                        self._log(
                            logging.WARNING,
                            'lerobot_medkit: predict_action_chunk is called outside '
                            'select_action (chunk or RTC inference): the step of each chunk '
                            'is unknown, so chunks are not published',
                        )
            self._flush_logs()
            return result

        return predict_action_chunk

    def _note_chunk_in_step(self, called):
        """Warn once when select_action never reaches predict_action_chunk.

        SmolVLA, gaussian_actor, lingbot_va and longwam (lerobot 0.7) run their
        model from select_action without predict_action_chunk: no chunk and no
        inference time reaches the monitor.
        """
        if called:
            self._chunk_in_step_seen = True
            return
        if self._chunk_in_step_seen or self._warned_no_chunk:
            return
        self._steps_without_chunk += 1
        steps = getattr(getattr(self._policy, 'config', None), 'n_action_steps', None)
        limit = 2 * steps if isinstance(steps, int) and steps > 0 else 200
        if self._steps_without_chunk >= limit:
            self._warned_no_chunk = True
            self._log(
                logging.WARNING,
                f'lerobot_medkit: {type(self._policy).__name__}.select_action ran '
                f'{self._steps_without_chunk} times without calling predict_action_chunk: '
                'no chunks are published and slow_inference measure: chunk sees nothing. '
                'Use measure: step for this policy',
            )

    def _guard(self, where, fn, *args):
        try:
            fn(*args)
        except Exception:  # noqa: B902 - the monitor must never break the policy
            count = self._failures.get(where, 0)
            self._failures[where] = count + 1
            if count == 0:
                self._log(
                    logging.ERROR, f'lerobot_medkit: {where} failed (logged once)', sys.exc_info()
                )

    def _log(self, level, message, exc_info=None):
        """Queue a log line; called with the lock held."""
        self._logs.append((level, message, exc_info))

    def _flush_logs(self):
        """Write queued log lines, outside the lock."""
        with self._lock:
            logs, self._logs = self._logs, []
        for level, message, exc_info in logs:
            logger.log(level, message, exc_info=exc_info)

    # What each call means

    def _on_reset(self):
        self.episode += 1
        self.step = 0
        self._slow.reset_window()

    def _publish(self, result, elapsed_ms, stamp):
        chunk = _actions(result)
        if chunk is None:
            return
        if self._postprocess is not None:
            chunk = self._postprocess(chunk)
        self._transport.publish_introspection(
            self.episode, self.step, _to_numpy(chunk), elapsed_ms, stamp=stamp
        )

    def _check_slow(self, elapsed_ms):
        change = self._slow.update(elapsed_ms)
        if change is None:
            return
        code = self._config.code('INFERENCE_SLOW')
        if change:
            what = 'steps' if self._config.slow_inference.measure == 'step' else 'inferences'
            self._transport.report_fault(
                code,
                self._config.slow_inference.severity,
                f'mean time {self._slow.mean_ms:.0f} ms over the last {self._slow.window} '
                f'{what}, limit {self._slow.max_ms:.0f} ms',
            )
        else:
            self._transport.clear_fault(code)

    def _camera_check(self, key):
        if key in self._cameras:
            return self._cameras[key]
        code = self._config.code(f'CAMERA_{_camera_name(key)}_FROZEN')
        other = self._camera_codes.get(code)
        if other is not None:
            self._log(
                logging.ERROR,
                f'lerobot_medkit: {other!r} and {key!r} both map to {code}: '
                f'{key!r} is not checked',
            )
            self._cameras[key] = None
            return None
        self._camera_codes[code] = key
        check = FrozenFrames(self._config.frozen_camera.identical_frames, _frames_equal)
        check.code = code
        self._cameras[key] = check
        return check

    def _check_frames(self, batch):
        if self._config.frozen_camera.identical_frames <= 0 or not isinstance(batch, dict):
            return
        for key, frame in batch.items():
            if not key.startswith(IMAGE_PREFIX) or not hasattr(frame, 'shape'):
                continue
            check = self._camera_check(key)
            if check is None:
                continue
            # Keep a copy: the caller may reuse the buffer for the next frame.
            change = check.update(frame.clone() if hasattr(frame, 'clone') else frame.copy())
            if change is None:
                continue
            if change:
                self._transport.report_fault(
                    check.code,
                    self._config.frozen_camera.severity,
                    f'{key}: {check.identical_frames} identical frames in a row, '
                    'as the policy received them',
                )
            else:
                self._transport.clear_fault(check.code)


def make_transport(config, node=None):
    """Build the transport `config` names; `node`: an rclpy node to report through."""
    if config.transport == 'none':
        from lerobot_medkit.transport import NullTransport

        return NullTransport()
    from lerobot_medkit.rclpy_transport import RclpyTransport

    return RclpyTransport(
        node=node,
        node_name=config.node_name,
        source_id=config.source_id,
        report_service=config.report_service,
        introspection_topic=config.introspection.topic if config.introspection.enabled else '',
    )


def attach(policy, config, transport=None, postprocess=None, node=None):
    """Monitor `policy`. `config`: a MedkitConfig, a dict, or a YAML path."""
    from lerobot_medkit import config as config_module

    if isinstance(config, str):
        config = config_module.load(config)
    elif isinstance(config, dict):
        config = config_module.from_dict(config)
    if transport is None:
        transport = make_transport(config, node=node)
    return PolicyMonitor(policy, config, transport, postprocess=postprocess).attach()
