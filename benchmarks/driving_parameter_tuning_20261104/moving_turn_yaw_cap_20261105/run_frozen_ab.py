#!/usr/bin/env python3
"""Bench-only isolated actual DWB comparison; no Gazebo or driving claim."""
import gzip
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
REPLAY = REPO/'scripts/replay_incident_controller.py'
BASE = REPO/'ros2_ws/src/smart_vision_amr/config/classroom_demo_operator_20261104.yaml'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_replay():
    spec = importlib.util.spec_from_file_location('moving_yaw_replay', REPLAY)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original = module.summarize_evaluation
    def summarize(evaluation):
        result = original(evaluation)
        valid = [item['traj']['velocity'] for item in evaluation.get('twists', [])
                 if item.get('total', -1.) >= 0.]
        moving = [v for v in valid if math.hypot(v['x'], v['y']) > .01+1e-9]
        result.update(valid_moving_over_035=sum(abs(v['theta'])>.35+1e-9 for v in moving),
                      max_valid_moving_yaw=max((abs(v['theta']) for v in moving), default=None))
        return result
    module.summarize_evaluation = summarize
    return module


def main():
    if len(sys.argv)>1 and sys.argv[1]=='--single':
        sys.argv=[str(REPLAY),*sys.argv[2:]]
        load_replay().main()
        return
    destination = Path(sys.argv[1]).resolve() if len(sys.argv)>1 else HERE/'recorded_runs'
    if destination.exists():
        raise SystemExit('Refusing existing output '+str(destination))
    destination.mkdir(parents=True)
    inputs = REPO/'benchmarks/driving_parameter_tuning_20261104/controller_screen/inputs'
    cases=[('folded_physical_incident',inputs/'folded_physical_incident.jsonl.gz',1793781696.5),
           ('corner',inputs/'corner.jsonl',1.)]
    library=next(Path(prefix)/'lib/libroute_tracking_dwb.so'
                 for prefix in os.environ.get('AMENT_PREFIX_PATH','').split(os.pathsep)
                 if prefix and (Path(prefix)/'lib/libroute_tracking_dwb.so').is_file())
    manifest={'scope':'Frozen open-loop actual Nav2, no Gazebo/closed-loop/physical/stop-QA pass',
              'costmap':'Reconstructed from identical captured scan, not captured costmap history',
              'comparison':'.50 cap matches previous finite candidate range; .35 rejects translating yaw only',
              'domain':186,'localhost_only':True,'duration_seconds':2.,
              'output_topic':'/replay/controller_cmd_vel','profile_sha256':sha(BASE),
              'plugin_sha256':sha(library),'runner_sha256':sha(Path(__file__)),
              'replay_sha256':sha(REPLAY),'inputs':{name:sha(path) for name,path,_ in cases}}
    (destination/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    rows=[]
    for cap in (.5,.35):
        for name,path,stamp in cases:
            output=destination/f'cap{round(cap*100):02d}'/name
            command=[sys.executable,str(Path(__file__).resolve()),'--single',str(path),
                     '--at-time',str(stamp),'--nav-params',str(BASE),'--output',str(output),
                     '--duration','2','--domain','186','--override',f'FollowPath.moving_turn_max_angular={cap}']
            done=subprocess.run(command,capture_output=True,text=True,
                                env={**os.environ,'ROS_DOMAIN_ID':'186','ROS_LOCALHOST_ONLY':'1'},cwd=REPO)
            output.mkdir(parents=True,exist_ok=True)
            (output/'runner.log').write_text(done.stdout+done.stderr)
            if done.returncode:
                raise RuntimeError(str(output)+' replay failed')
            events=[json.loads(line) for line in (output/'observations.jsonl').read_text().splitlines()]
            evaluations=[x['data'] for x in events if x['kind']=='evaluation' and x['data'].get('best')]
            if not evaluations:
                raise RuntimeError(str(output)+' no legal evaluation')
            winner=evaluations[-1]['best']['velocity']
            rows.append({'cap_radps':cap,'case':name,'evaluations':len(evaluations),
                         'forward_valid_last':evaluations[-1]['forward_valid_count'],
                         'winner':winner,'valid_moving_over_035':sum(x['valid_moving_over_035'] for x in evaluations),
                         'max_valid_moving_yaw':max(x['max_valid_moving_yaw'] for x in evaluations),
                         'invalid_by_critic_last':evaluations[-1]['invalid_by_critic'],
                         'selected_pure_yaw':sum(abs(x['best']['velocity']['x'])<1e-9 and
                                                 abs(x['best']['velocity']['theta'])>.08 for x in evaluations)})
            with gzip.open(output/'observations.jsonl.gz','wt') as archived:
                archived.write((output/'observations.jsonl').read_text())
            (output/'observations.jsonl').unlink()
            print(json.dumps(rows[-1]),flush=True)
            (destination/'results.json').write_text(json.dumps(rows,indent=2)+'\n')


if __name__=='__main__':
    main()
