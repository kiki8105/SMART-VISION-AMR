#!/usr/bin/env python3
"""Exercise SLAM shutdown with fake ROS processes; never start Gazebo."""

import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import unittest


RUN_SCRIPT = Path(__file__).resolve().with_name("run_sim.sh")

FAKE_ROS = r'''#!/usr/bin/python3
import json
import os
import signal
import sys
import time

def record(name, **fields):
    fields.update(event=name, pid=os.getpid(), time=time.monotonic())
    with open(os.environ["TEST_EVENTS"], "a") as stream:
        stream.write(json.dumps(fields) + "\n")

def stop(signum, _frame):
    record("launch_exit", signal=signum)
    sys.exit(0)

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
record("launch_start", argv=sys.argv[1:],
       workspace=os.environ.get("TEST_SOURCED_WORKSPACE"))
while True:
    time.sleep(0.02)
'''

FAKE_SAVE = r'''#!/usr/bin/python3
import json
import os
from pathlib import Path
import sys
import time

def record(name, **fields):
    fields.update(event=name, pid=os.getpid(), time=time.monotonic())
    with open(os.environ["TEST_EVENTS"], "a") as stream:
        stream.write(json.dumps(fields) + "\n")

counter = Path(os.environ["TEST_COUNTER"])
attempt = int(counter.read_text()) + 1 if counter.exists() else 1
counter.write_text(str(attempt))
prefix = Path(sys.argv[-1])
record("save_start", argv=sys.argv[1:], attempt=attempt, prefix=str(prefix))
time.sleep(float(os.environ.get("TEST_SAVE_DELAY", "0.05")))
mode = os.environ.get("TEST_SAVE_MODE", "success")
if mode == "fail_first" and attempt == 1:
    record("save_failed", attempt=attempt)
    sys.exit(1)
prefix.parent.mkdir(parents=True, exist_ok=True)
Path(str(prefix) + ".yaml").write_text("image: map.pgm\nresolution: 0.05\n")
if mode != "missing_first" or attempt != 1:
    Path(str(prefix) + ".pgm").write_bytes(b"P2\n1 1\n255\n254\n")
record("save_done", attempt=attempt)
'''


class FakeSimulation:
    """A disposable copy of the entry point and independently signalled launch."""

    def __init__(self, arguments, *, mode="success", save_delay=0.05,
                 custom_workspace=False, explicit_prefix=False):
        self.temp = tempfile.TemporaryDirectory(prefix="sim-autosave-test-")
        self.root = Path(self.temp.name)
        self.workspace = self.root / "workspace"
        scripts = self.workspace / "src" / "SMART-VISION-AMR" / "scripts"
        scripts.mkdir(parents=True)
        shutil.copy2(RUN_SCRIPT, scripts / "run_sim.sh")
        config = scripts.parent / "ros2_ws" / "src" / "smart_vision_amr" / "config"
        config.mkdir(parents=True)
        for name in ("simulation.yaml", "simulation_nodes.yaml"):
            (config / name).write_text("{}\n")
        self.parameter_arguments = [
            f"nav_params_file:={config / 'simulation.yaml'}",
            f"nodes_params_file:={config / 'simulation_nodes.yaml'}",
        ]
        (scripts / "sim_env.bash").write_text(
            'export TEST_SOURCED_WORKSPACE="$1"\n'
            'export PATH="$TEST_BIN:$PATH"\n'
        )
        (scripts / "save_sim_map.sh").write_text(FAKE_SAVE)
        (scripts / "save_sim_map.sh").chmod(0o755)
        binary_directory = self.root / "bin"
        binary_directory.mkdir()
        (binary_directory / "ros2").write_text(FAKE_ROS)
        (binary_directory / "ros2").chmod(0o755)
        self.events_path = self.root / "events.jsonl"
        self.events_path.touch()
        self.requested_workspace = (
            self.root / "custom workspace" if custom_workspace else self.workspace
        )
        self.requested_workspace.mkdir(exist_ok=True)
        options = []
        if custom_workspace:
            options += ["--workspace", str(self.requested_workspace)]
        self.explicit_prefix = self.root / "saved maps" / "custom_map"
        if explicit_prefix:
            options += ["--map-prefix", str(self.explicit_prefix)]
        environment = os.environ.copy()
        environment.update(
            TEST_BIN=str(binary_directory),
            TEST_EVENTS=str(self.events_path),
            TEST_COUNTER=str(self.root / "attempts"),
            TEST_SAVE_MODE=mode,
            TEST_SAVE_DELAY=str(save_delay),
        )
        self.output_path = self.root / "output.txt"
        self.output = self.output_path.open("w+")
        self.process = subprocess.Popen(
            ["bash", str(scripts / "run_sim.sh"), *options, *arguments],
            env=environment,
            stdout=self.output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def events(self):
        # An append may be observed between write and newline; retry that record.
        result = []
        for line in self.events_path.read_text().splitlines():
            try:
                result.append(json.loads(line))
            except json.JSONDecodeError:
                pass
        return result

    def wait_event(self, name, *, occurrence=1, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            matches = [event for event in self.events() if event["event"] == name]
            if len(matches) >= occurrence:
                return matches[occurrence - 1]
            if self.process.poll() is not None:
                break
            time.sleep(0.02)
        raise AssertionError(
            f"Missing event {name!r} #{occurrence}; "
            f"exit={self.process.poll()}; events={self.events()}; "
            f"output={self.output_path.read_text()}"
        )

    def interrupt(self):
        # A terminal's Ctrl+C targets its foreground group, not just the shell.
        os.killpg(self.process.pid, signal.SIGINT)

    def wait_exit(self):
        try:
            return self.process.wait(timeout=5)
        except subprocess.TimeoutExpired as error:
            raise AssertionError(
                f"Runner did not exit: {self.output_path.read_text()}"
            ) from error

    def close(self):
        # setsid detaches launch and saver; clean both even after a failed test.
        for event in self.events():
            if event["event"] in ("launch_start", "save_start"):
                try:
                    process_group = os.getpgid(event["pid"])
                    if process_group != os.getpgrp():
                        os.killpg(process_group, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        try:
            os.killpg(self.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.process.wait(timeout=5)
        self.output.close()
        self.temp.cleanup()


class RunSimAutosaveTests(unittest.TestCase):
    def runner(self, arguments, **options):
        simulation = FakeSimulation(arguments, **options)
        self.addCleanup(simulation.close)
        simulation.wait_event("launch_start")
        return simulation

    def assert_saved_before_launch_exit(self, simulation):
        self.assertEqual(simulation.wait_exit(), 0,
                         simulation.output_path.read_text())
        events = simulation.events()
        saved = next(event for event in events if event["event"] == "save_done")
        stopped = next(event for event in events if event["event"] == "launch_exit")
        self.assertLess(saved["time"], stopped["time"])
        prefix = Path(next(event for event in events
                           if event["event"] == "save_start")["prefix"])
        for suffix in ("yaml", "pgm"):
            self.assertGreater(Path(f"{prefix}.{suffix}").stat().st_size, 0)
        return prefix

    def test_ctrl_c_saves_default_map_before_stopping_launch(self):
        simulation = self.runner(["slam:=true"])
        simulation.interrupt()
        prefix = self.assert_saved_before_launch_exit(simulation)
        self.assertEqual(prefix.name, "map")
        self.assertEqual(prefix.parent.parent, simulation.workspace / "maps")
        self.assertTrue(prefix.parent.name.startswith("slam_"))

    def test_failed_save_keeps_launch_alive_and_next_ctrl_c_retries(self):
        simulation = self.runner(["slam:=true"], mode="fail_first")
        simulation.interrupt()
        simulation.wait_event("save_failed")
        time.sleep(0.15)
        self.assertIsNone(simulation.process.poll())
        self.assertNotIn("launch_exit", [event["event"] for event in simulation.events()])
        simulation.interrupt()
        simulation.wait_event("save_start", occurrence=2)
        self.assert_saved_before_launch_exit(simulation)

    def test_success_without_both_map_artifacts_keeps_launch_alive(self):
        simulation = self.runner(["slam:=true"], mode="missing_first")
        simulation.interrupt()
        simulation.wait_event("save_done")
        time.sleep(0.15)
        self.assertIsNone(simulation.process.poll())
        self.assertNotIn("launch_exit", [event["event"] for event in simulation.events()])
        simulation.interrupt()
        simulation.wait_event("save_start", occurrence=2)
        self.assertEqual(simulation.wait_exit(), 0, simulation.output_path.read_text())
        events = simulation.events()
        saved = [event for event in events if event["event"] == "save_done"][-1]
        stopped = next(event for event in events if event["event"] == "launch_exit")
        self.assertLess(saved["time"], stopped["time"])

    def test_repeated_ctrl_c_during_save_does_not_interrupt_it(self):
        simulation = self.runner(["slam:=true"], save_delay=0.6)
        simulation.interrupt()
        simulation.wait_event("save_start")
        simulation.interrupt()
        time.sleep(0.05)
        simulation.interrupt()
        self.assert_saved_before_launch_exit(simulation)
        self.assertEqual(len([event for event in simulation.events()
                              if event["event"] == "save_start"]), 1)

    def test_workspace_and_explicit_prefix_reach_save_script(self):
        simulation = self.runner(
            ["slam:=true", "example:=value"],
            custom_workspace=True,
            explicit_prefix=True,
        )
        start = simulation.wait_event("launch_start")
        self.assertEqual(start["workspace"], str(simulation.requested_workspace))
        self.assertEqual(start["argv"], ["launch", "smart_vision_amr",
                                        "simulation.launch.py", *simulation.parameter_arguments,
                                        "slam:=true", "example:=value"])
        simulation.interrupt()
        self.assertEqual(self.assert_saved_before_launch_exit(simulation),
                         simulation.explicit_prefix)
        saved = simulation.wait_event("save_start")
        self.assertEqual(saved["argv"], ["--workspace",
                                         str(simulation.requested_workspace),
                                         str(simulation.explicit_prefix)])

    def test_non_slam_launches_do_not_attempt_save(self):
        for arguments in ([], ["slam:=false"], ["slam:=true", "slam:=false"]):
            with self.subTest(arguments=arguments):
                simulation = self.runner(arguments)
                simulation.interrupt()
                self.assertEqual(simulation.wait_exit(), 0,
                                 simulation.output_path.read_text())
                self.assertNotIn("save_start", [event["event"]
                                                for event in simulation.events()])

    def test_last_slam_value_accepts_uppercase_true_and_one(self):
        for arguments in (["slam:=false", "slam:=TRUE"], ["slam:=1"]):
            with self.subTest(arguments=arguments):
                simulation = self.runner(arguments)
                simulation.interrupt()
                self.assert_saved_before_launch_exit(simulation)


if __name__ == "__main__":
    unittest.main()
