"""Normal navigation process lifecycle. Development acceptance is a separate tool."""
from pathlib import Path
import argparse,datetime as dt,fcntl,json,math,os,signal,subprocess,time,uuid
from embodied_navigation.common.config import ROOT, read, path


def environment(directory,domain,profile,socket):
    from embodied_navigation.common.runtime_environment import environment as ros_environment
    from embodied_navigation.common.communication.dds_environment import configure
    env=configure(ros_environment('/opt/ros/jazzy/setup.bash'))
    transport=profile['dds_transport']
    for key in transport['remove_environment_keys']:env.pop(key,None)
    env.update(transport['environment'])
    env['FASTRTPS_DEFAULT_PROFILES_FILE']=str(ROOT/'src/embodied_navigation/common/communication/dds_transport.xml')
    env.update(ROS_DOMAIN_ID=str(domain),GZ_IP='127.0.0.1',GZ_PARTITION='navigation_'+directory.name,
        TEACHER_SOCKET=socket,TEACHER_ACTUATOR_LOG='/dev/null',
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(ROOT/'install/actuators/lib'),DEMO_RUN_DIR=str(directory),
        ROS_LOG_DIR=str(directory/'ros_logs'),PYTHONDONTWRITEBYTECODE='1',
        OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',
        FASTLIVO_PIPELINE_MODE=profile['pipeline']['mode'],FASTLIVO_IMAGE_COPY_OPT=str(profile['pipeline']['image_copy_opt']),
        FASTLIVO_SERIAL_IMU_PRIORITY='1',FASTLIVO_LIO_JACOBIAN_THREADS='4',FASTLIVO_VIO_PATCH_THREADS='4',
        FASTLIVO_SURFACE_VALIDITY='0',NAVIGATION_RECORD_RAW='0')
    env.update(profile.get('features',{}))
    for key in ['R87_DIAGNOSTIC_PROBE','R89_THREAD_PROBE','R96_CONTROL_CPU_PROBE','R99_CLOCK_PROBE',
        'R107_SCAN_TRANSPORT_PROBE','R118_SOURCE_AGE_PROBE','MATCH_QUERY_CAPTURE']:env[key]='0'
    return env


def preflight_resources():
    # Explicit developer option; no historical storage ledger or authority file.
    if os.getloadavg()[0]>.9*(os.cpu_count()or 1):raise RuntimeError('CPU load protection: launch postponed')
    fields={line.split(':',1)[0]:line.split(':',1)[1].strip()for line in Path('/proc/meminfo').read_text().splitlines()}
    if int(fields['MemAvailable'].split()[0])*1024<4*1024**3:raise RuntimeError('Available memory below 4GiB')


def parse_arguments(argv=None):
    """Reject invalid launch limits before configuration or runtime side effects."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true');parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--domain',type=int,default=231);parser.add_argument('--label',default='navigation')
    parser.add_argument('--resource-check',action='store_true',help='Opt-in developer resource precheck')
    parser.add_argument('--duration',type=float,default=None,help='Optional finite positive wall-clock run limit in seconds')
    args=parser.parse_args(argv)
    if not 1<=args.domain<=232:parser.error('Domain must be in 1..232')
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration<=0):
        parser.error('Duration must be finite and positive')
    return args


def main(argv=None):
    args=parse_arguments(argv)
    profile=read()
    from embodied_navigation.launch.preflight import check
    dependencies=check()
    if not (args.check or args.prepare_only) and not dependencies['ready']:
        raise RuntimeError('Missing runtime dependencies; run start.py --check for the full list')
    if args.resource_check:preflight_resources()
    if args.check:
        print(json.dumps(dependencies,ensure_ascii=False,indent=2));return 0 if dependencies['ready'] else 2
    locks=[];children=[];logs=[];stop=False;failure=None
    try:
        for name in ['go2_pid_physics_serial.lock','go2_model_pid_simulation.lock','go2_navigation_dispatch.lock']:
            stream=(Path('/tmp')/name).open('a+');fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB);locks.append(stream)
        stamp=dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        label=''.join(c for c in args.label if c.isalnum()or c in '_-')or'navigation'
        from embodied_navigation.launch.prepare import prepare
        directory=ROOT/'runs'/(stamp+'_'+label+'_'+uuid.uuid4().hex[:4]);profile=prepare(directory,profile)
        print(json.dumps(dict(stage='prepared',directory=str(directory),acceptance_required=False)),flush=True)
        if args.prepare_only:return 0
        socket='/tmp/navigation_'+uuid.uuid4().hex[:12]+'.sock';env=environment(directory,args.domain,profile,socket)
        def requested_stop(signum,frame):
            nonlocal stop;stop=True
        previous={sig:signal.signal(sig,requested_stop)for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]}
        def start(role,command,overrides=None):
            log=(directory/(role+'.log')).open('x');logs.append(log)
            process=subprocess.Popen(command,cwd=ROOT,env={**env,**(overrides or{})},stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            raw=Path('/proc',str(process.pid),'stat').read_text();ticks=int(raw[raw.rfind(')')+2:].split()[19])
            children.append((role,process,ticks));(directory/'processes.json').write_text(json.dumps([dict(role=n,pid=p.pid,start_ticks=t)for n,p,t in children],indent=2)+'\n')
            return process
        start('launch',['ros2','launch',str(ROOT/'src/embodied_navigation/launch/launch/demo.launch.py'),'run_dir:='+str(directory)])
        print(json.dumps(dict(stage='running',directory=str(directory),pid=os.getpid(),acceptance_required=False)),flush=True)
        began=time.monotonic()
        while not stop:
            for role,process,_ in children:
                if process.poll()is not None:raise RuntimeError(role+' exited with '+str(process.returncode))
            if args.duration is not None and time.monotonic()-began>=args.duration:break
            time.sleep(.2)
        return 0
    except BaseException as error:
        failure=type(error).__name__+': '+str(error);raise
    finally:
        # Only process groups created here are signalled; PID start time guards reuse.
        for _,process,ticks in reversed(children):
            try:
                raw=Path('/proc',str(process.pid),'stat').read_text();actual=int(raw[raw.rfind(')')+2:].split()[19])
                # Launch owns component children and forwards one signal.
                # A group signal here would reach children a second time when
                # launch forwards SIGINT, interrupting their finally blocks.
                if actual==ticks:process.send_signal(signal.SIGINT)
            except (OSError,ProcessLookupError):pass
        deadline=time.monotonic()+15
        for _,process,_ in reversed(children):
            try:process.wait(timeout=max(.1,deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                try:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
                except (OSError,subprocess.TimeoutExpired):pass
        for log in logs:log.close()
        for stream in locks:stream.close()
        if children:
            from embodied_navigation.launch.exit_status import failed_exits
            process_codes = {role: p.poll() for role, p, _ in children}
            component_file = directory/'component_exit.jsonl'
            component_exits = [json.loads(line) for line in component_file.read_text().splitlines() if line.strip()] if component_file.exists() else []
            unsuccessful = failed_exits(process_codes, component_exits)
            cleanup_error = 'Managed process shutdown failed: '+json.dumps(unsuccessful) if unsuccessful and failure is None else None
            failure = failure or cleanup_error
            (directory/'exit.json').write_text(json.dumps(dict(error=failure,
                process_exit_codes=process_codes,component_exits=component_exits),indent=2)+'\n')
            print(json.dumps(dict(stage='stopped',directory=str(directory),error=failure)),flush=True)
            if cleanup_error:
                raise RuntimeError(cleanup_error)
