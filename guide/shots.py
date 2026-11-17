"""Take a guide's screenshots: run the app (its own Python) with p4core.guide.capture.

The guide's front matter names the app:
    app_main:   puckutility.gui.app:main     module:function that starts the GUI
    app_args:   --can vcan0                  its arguments (the SIL simulator's bus)
    app_root:   ../..                        the app's source tree, relative to guide.md
    app_python: ../../.venv/bin/python       optional; else $P4GUIDE_APP_PYTHON, else this Python
Shots are the ```shot NAME``` blocks (see capture.py for the step syntax).
--sim PATH starts that puck_sim on the app's bus (fresh config) for the capture, then stops it.
"""
import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time

from . import build


def take(guide_path, only=None, sim=None, sim_config=None, timeout=600):
    meta, _, shots = build.read_guide(guide_path)
    gdir = os.path.dirname(os.path.abspath(guide_path))
    out = os.path.join(gdir, 'shots')
    root = os.path.normpath(os.path.join(gdir, meta.get('app_root', '.')))
    py = os.environ.get('P4GUIDE_APP_PYTHON') or (
        os.path.normpath(os.path.join(gdir, meta['app_python'])) if meta.get('app_python') else sys.executable)
    names = [n for n in shots if not only or n in only]
    if not names:
        raise SystemExit('no ```shot``` blocks to take' + (' matching ' + ', '.join(only) if only else ''))
    work = tempfile.mkdtemp(prefix='p4guide-shots-')
    spec = {'main': meta['app_main'], 'args': shlex.split(meta.get('app_args', '')), 'out': out,
            'startup_s': float(meta.get('startup_s', 3)), 'timeout_s': timeout,
            'shots': [{'name': n, 'steps': shots[n]} for n in names]}
    json.dump(spec, open(os.path.join(work, 'spec.json'), 'w'))
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(
        [root] +
        ([os.environ['PYTHONPATH']] if os.environ.get('PYTHONPATH') else [])))
    if sys.platform.startswith('linux'):
        env.setdefault('GDK_BACKEND', 'x11')        # client-area capture needs X11 (XWayland is fine)
    proc_sim = None
    if sim:
        bus = next((a for a in spec['args'] if a.startswith('vcan')), 'vcan0')
        senv = dict(os.environ, SIM_CAN_IF=bus, SIM_NODE_ID=str(meta.get('sim_node', 127)),
                    SIM_NVM=os.path.join(work, 'nvm.bin'))
        proc_sim = subprocess.Popen([os.path.abspath(sim)], cwd=os.path.dirname(os.path.abspath(sim)), env=senv,
                                    stdout=open(os.path.join(work, 'sim.log'), 'w'), stderr=subprocess.STDOUT)
        time.sleep(1.5)
        if proc_sim.poll() is not None:
            msg = open(os.path.join(work, 'sim.log')).read()[-500:]
            shutil.rmtree(work, ignore_errors=True)
            raise SystemExit('puck_sim exited: ' + msg)
        print('puck_sim pid {} on {}'.format(proc_sim.pid, bus))
    for cmd in [c.strip() for c in meta.get('setup', '').split(';') if c.strip()]:
        # e.g. load a configuration into the simulator before the capture (the app's own CLI)
        cmd = cmd.replace('{python}', shlex.quote(py)).replace('{guide}', shlex.quote(gdir))
        print('setup:', cmd)
        rs = subprocess.run(cmd, shell=True, cwd=root, env=env)
        if rs.returncode != 0:
            if proc_sim is not None:
                proc_sim.kill()
            shutil.rmtree(work, ignore_errors=True)
            raise SystemExit('setup step failed (exit {}): {}'.format(rs.returncode, cmd))
    try:
        r = subprocess.run([py, '-u', '-c', 'import p4core.guide.capture as c; c.run({!r})'.format(
            os.path.join(work, 'spec.json'))], cwd=work, env=env, timeout=timeout + 30)   # the app's logs/ etc. land in the temp folder
    finally:
        if proc_sim is not None:
            proc_sim.send_signal(signal.SIGTERM)
            try:
                proc_sim.wait(5)
            except subprocess.TimeoutExpired:
                proc_sim.kill()
    shutil.rmtree(work, ignore_errors=True)          # spec, simulator NV and log: temporary
    if r.returncode != 0:
        raise SystemExit('capture failed (exit {}): see the log above'.format(r.returncode))
    return out
