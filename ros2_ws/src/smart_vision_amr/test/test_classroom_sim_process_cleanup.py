"""A failed isolated Gazebo trial must not leave another Nav2 owner running."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
import run_classroom_staged_validation as staged


def _stubborn_process(*, stdout=None):
    return subprocess.Popen(
        [sys.executable, '-c',
         'import signal,time; '
         'signal.signal(signal.SIGINT, signal.SIG_IGN); '
         'signal.signal(signal.SIGTERM, signal.SIG_IGN); '
         'print("ready", flush=True); time.sleep(30)'],
        start_new_session=True, stdout=stdout, text=True)


def _live_process(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1][0] != 'Z'
    except FileNotFoundError:
        return False


def test_fault_sim_cleanup_kills_only_its_stubborn_setsid_group():
    source = (ROOT/'scripts/run_classroom_fault_sim.sh').read_text()
    body = source.split('cleanup() {\n', 1)[1].split('\n}\ntrap cleanup EXIT', 1)[0]
    function = 'cleanup() {\n' + body + '\n}'
    owned = _stubborn_process(stdout=subprocess.PIPE)
    unrelated = _stubborn_process(stdout=subprocess.PIPE)
    try:
        assert owned.stdout.readline().strip() == 'ready'
        assert unrelated.stdout.readline().strip() == 'ready'
        command = (function + f'\nVERIFY_PID=\nLAUNCH_PID={owned.pid}\n'
                   'RELAY_PID=\nCONDITIONER_PID=\nBATTERY_PID=\nPAIR_PID=\n'
                   'cleanup\n')
        result = subprocess.run(['bash', '-e', '-c', command],
                                capture_output=True, text=True, timeout=9)
        assert result.returncode == 0, result.stderr
        assert owned.wait(timeout=1) == -signal.SIGKILL
        assert unrelated.poll() is None
    finally:
        for process in (owned, unrelated):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)


def test_fault_sim_term_exits_after_cleaning_owned_group():
    source = (ROOT/'scripts/run_classroom_fault_sim.sh').read_text()
    body, remaining = source.split('cleanup() {\n', 1)[1].split('\n}\n', 1)
    function = 'cleanup() {\n' + body + '\n}'
    traps = remaining.split('\n\nEKF_ARGS=', 1)[0]
    owned = _stubborn_process(stdout=subprocess.PIPE)
    shell = None
    try:
        assert owned.stdout.readline().strip() == 'ready'
        command = (function + f'\nVERIFY_PID=\nLAUNCH_PID={owned.pid}\n'
                   'RELAY_PID=\nCONDITIONER_PID=\nBATTERY_PID=\nPAIR_PID=\n'
                   + traps + '\nprintf "ready\\n"\n'
                   'while :; do sleep .1; done\n')
        shell = subprocess.Popen(['bash', '-e', '-c', command],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 text=True)
        assert shell.stdout.readline().strip() == 'ready'
        shell.send_signal(signal.SIGTERM)
        _, stderr = shell.communicate(timeout=9)
        assert shell.returncode == 143, stderr
        assert owned.wait(timeout=1) == -signal.SIGKILL
    finally:
        if shell is not None and shell.poll() is None:
            shell.kill()
            shell.wait(timeout=2)
        if owned.poll() is None:
            os.killpg(owned.pid, signal.SIGKILL)
            owned.wait(timeout=2)


def test_staged_cleanup_reaps_owned_group_after_leader_exits():
    # ros2 launch may exit while a lifecycle_manager child remains in its PGID.
    leader_code = (
        'import subprocess,sys; '
        'child=subprocess.Popen([sys.executable,"-c",'
        '"import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); '
        'signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(30)"],'
        'stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL); '
        'print(child.pid,flush=True)')
    leader = subprocess.Popen([sys.executable, '-c', leader_code],
                              start_new_session=True, stdout=subprocess.PIPE,
                              text=True)
    member_pid = int(leader.stdout.readline())
    try:
        leader.wait(timeout=2)
        assert os.getpgid(member_pid) == leader.pid
        assert _live_process(member_pid)
        staged.terminate_group(leader, timeout=.1)
        deadline = time.monotonic() + 2
        while _live_process(member_pid) and time.monotonic() < deadline:
            time.sleep(.02)
        assert not _live_process(member_pid)
    finally:
        if _live_process(member_pid):
            os.killpg(leader.pid, signal.SIGKILL)
        leader.wait(timeout=2)
