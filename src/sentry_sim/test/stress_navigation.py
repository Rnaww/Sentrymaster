"""在已 source 的工作空间执行完整 Gazebo 比赛；结果保存在 stress_results。"""
import argparse, hashlib, json, os, signal, statistics, subprocess, time
from pathlib import Path


def stop(process):
    exited = process.poll() is not None
    # launch 自己把 SIGINT 传给节点，不能再给整个组重复发一次。
    try: os.kill(process.pid, signal.SIGINT) if not exited else os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError: return
    if exited:
        time.sleep(2)
        try: os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError: pass
        return
    try: process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--count', type=int, default=100)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--timeout', type=float, default=600)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    if args.count < 1: parser.error('count must be positive')
    directory = args.output.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    sources = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
               for p in sorted((root/'src').rglob('*'))
               if p.is_file() and p.suffix in ('.py','.cpp','.hpp','.h','.yaml','.xml','.world','.urdf','.png') and '__pycache__' not in p.parts}
    metadata = {'head': subprocess.check_output(['git','rev-parse','HEAD'], cwd=root, text=True).strip(),
                'source_sha256': sources, 'count': args.count, 'physics': 'gazebo',
                'domain': args.domain, 'port': args.port, 'started': time.time()}
    (directory/'manifest.json').write_text(json.dumps(metadata, indent=2))
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), GAZEBO_MASTER_URI=f'http://127.0.0.1:{args.port}')
    results = []
    for number in range(1,args.count+1):
        changed = [p for p,digest in sources.items() if not (root/p).exists() or hashlib.sha256((root/p).read_bytes()).hexdigest() != digest]
        if changed: raise RuntimeError('Source changed during stress test: '+str(changed))
        run = directory/f'run_{number:03d}'
        run.mkdir()
        result_file = run/'result.json'
        start = time.monotonic()
        result = None
        status = 'timeout'
        with (run/'launch.log').open('w') as log:
            process = subprocess.Popen(['ros2','launch','sentry_sim','simulator_701.launch.py','physics:=gazebo',
                                        'gui:=false','rviz:=false','output_file:='+str(result_file)],
                                       cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                while time.monotonic()-start < args.timeout:
                    if result_file.exists():
                        try: result = json.loads(result_file.read_text())
                        except json.JSONDecodeError: pass
                        else:
                            status = 'completed'
                            break
                    if process.poll() is not None:
                        status = 'launch_exited'
                        break
                    time.sleep(1)
            finally: stop(process)
        row = {'run':number,'status':status,'wall_seconds':round(time.monotonic()-start,2)}
        log_text = (run/'launch.log').read_text(errors='replace')
        runtime_log, _, shutdown_log = log_text.partition('[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)')
        row['node_crashes'] = runtime_log.count('process has died')
        row['shutdown_exits'] = shutdown_log.count('process has died')
        row['controller_aborts'] = log_text.count('Aborting handle')
        row['lookahead_warnings'] = log_text.count('No traversable lookahead segment')
        if result is not None: row.update({k:v for k,v in result.items() if k not in ('trajectory','trajectory_columns')})
        results.append(row)
        scores = [r['score'] for r in results if r.get('status') == 'completed' and 'score' in r]
        summary = {'requested':args.count,'finished':len(results),'completed':sum(r['status']=='completed' for r in results),
                   'failed':sum(r['status']!='completed' for r in results),'full_score_runs':sum(s==585 for s in scores),
                   'score_min':min(scores) if scores else None,'score_mean':statistics.mean(scores) if scores else None,
                   'score_max':max(scores) if scores else None,'runs':results}
        temporary = directory/'summary.tmp'
        temporary.write_text(json.dumps(summary,ensure_ascii=False,indent=2))
        temporary.replace(directory/'summary.json')
        print(json.dumps(row,ensure_ascii=False),flush=True)


if __name__ == '__main__': main()
