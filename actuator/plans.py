"""plans.py -- the actuator calibration as the apps offer it, and running it.

A manufacturing batch shares one configuration CSV, made once on a pilot
actuator by full identification (p4sysid, pucktuner) and exported.  Each
actuator of the batch then gets the CSV (flash, set-id, config load,
Save.All, reboot: before any step) and the actuator calibration: the steps
whose results differ unit to unit and that the CSV does not carry.

  ACTUATOR  the per-actuator calibration:
              encoder          the sensor at rest; its chip and filter
              bias, gain       current-sense offsets and gain
              slope, fold      the shunt amplifier's current-proportional
                               offset and its drive-on baseline
              winding          electrical zero and winding order
              runout           the magnet's runout table
              friction         friction calibration: the drag's J, B and
                               Tc (the velocity gains rescaled to its J),
                               then the Coulomb feed-forward on that Tc:
                               before cogging, whose slow spins it carries
              cogging          the cogging feed-forward bins

Slope and fold belong to the board as much as bias and gain do: the CSV
loader skips their rows and export never writes them
(p4core.config_csv.BOARD_CALIBRATION), so a batch CSV cannot give them.
The cogging bins are not in the templates either.

by_stage() lists any selection under the dependency stages (prereqs.stages);
a selection always runs in steps.STEPS order (Session.run).

make_session() and run() are what the apps call: no windows here, progress
goes to a p4core Reporter.  Each step's results are saved to NV as soon as it
passes (steps.py), so a later failure does not lose them; session.unsaved is
then only set when that Save.App + Save.Mfg failed, which save() can try again.
"""

from . import prereqs, steps

FINAL = ('save', 'export')
ACTUATOR = ('encoder', 'bias', 'gain', 'slope', 'fold', 'winding', 'runout',
            'friction', 'cogging')

#: name -> (menu title, steps)
PLANS = {
    'actuator': ('Actuator Calibration', ACTUATOR),
}

#: Other names the apps' command lines take for a step.  offsets,
#: gainfactor and encoder-offset are the names a firmware A/B gives its four
#: calibration steps (current offsets, gainfactor, encoder offset, electrical
#: zero + winding order): the last two are the one winding sweep.
ALIASES = {'enczero': 'winding', 'ezero': 'winding', 'encoder-test': 'encoder',
           'encdir': 'winding', 'runout-table': 'runout',
           'drag': 'friction', 'friction-ff': 'friction',
           'friction_ff': 'friction',
           'offsets': 'bias', 'gainfactor': 'gain', 'encoder-offset': 'winding'}

#: Names that stand for several steps.  ab: the four steps of the 4.3.5 /
#: 5.0 A/B, which 4.3.x runs as well.
GROUPS = {'ab': ('bias', 'gain', 'winding')}


class PlanError(ValueError):
    pass


def _by_key():
    return {st.key: st for st in steps.STEPS}


def step(key):
    return _by_key()[key]


def title(key):
    return _by_key()[key].title


def canonical(name, allowed=None):
    """A step key from *name* (a key or an ALIASES name), one of *allowed*
    (default: every step in steps.STEPS)."""
    allowed = steps.STEP_KEYS if allowed is None else allowed
    key = ALIASES.get(name, name).replace('-', '_')
    if key not in allowed:
        raise PlanError('unknown step {!r}; choose from {}'.format(
            name, ', '.join(allowed)))
    return key


def expand(names, allowed=None):
    """Step keys for *names* (keys, ALIASES or GROUPS names), each once, in
    run order."""
    keys = set()
    for name in names:
        if name in GROUPS:
            keys.update(canonical(k, allowed) for k in GROUPS[name])
        else:
            keys.add(canonical(name, allowed))
    return run_order(keys)


def stage_of(key):
    """The dependency stage of *key* (1..), or None for Save and Export."""
    st = prereqs.stages()
    return st[key][0] if key in st else None


def by_stage(keys=None):
    """[(stage, [Step, ...]), ...] for *keys*: ascending stage, then STEPS
    order within a stage; Save and Export last under stage None."""
    st = prereqs.stages()
    order = steps.STEP_KEYS
    keys = order if keys is None else [k for k in order if k in set(keys)]
    by_key = _by_key()
    groups = {}
    for k in sorted(keys, key=lambda k: (st[k][0] if k in st else 1 << 30,
                                         order.index(k))):
        groups.setdefault(st[k][0] if k in st else None, []).append(by_key[k])
    return list(groups.items())


def numbered(keys=None):
    """[(n, stage, Step), ...]: by_stage() numbered from 1 in that order --
    the enumerated list the menus show."""
    out = []
    for stage, sts in by_stage(keys):
        for st in sts:
            out.append((len(out) + 1, stage, st))
    return out


def stage_label(stage):
    return 'Save' if stage is None else 'Stage {}'.format(stage)


def unavailable(node, key):
    """Why *node*'s firmware cannot run step *key* (steps.NEEDS), or None.
    The apps leave such a step out of what they offer."""
    return steps.unavailable(node, canonical(key))


def available_keys(node, keys):
    """The keys of *keys* that *node*'s firmware can run."""
    return tuple(k for k in keys if unavailable(node, k) is None)


def run_order(keys):
    """*keys* in the order Session.run runs them (steps.STEPS)."""
    keys = set(keys)
    return [k for k in steps.STEP_KEYS if k in keys]


# ---------------------------------------------------------------------------
# Running
# ---------------------------------------------------------------------------
def make_session(node, reporter, sync=None, reboot=None, limits=None,
                 export_path=None, device_class=None, **session_kw):
    """A steps.Session on *node* that logs to *reporter* (a p4core
    Reporter: note, should_abort) and stops when it asks.  *sync* is whoever
    owns the SYNC producer (p4gui's model); *reboot()* restarts the drive
    and returns the node (p4core Session.reset_node), without it a pole,
    PWM or encoder-filter change stops with 'reboot and run again'.
    *limits*: a commission.Limits (default: the CLI's defaults).
    *device_class* is the drive's class (default P4Drive; p4sysid's steps
    need its P4Sysid)."""
    from .commission import Limits
    from .device import P4Drive
    dev = (device_class or P4Drive)(node, log=reporter.note,
                                    should_abort=reporter.should_abort,
                                    sync=sync,
                                    detail=getattr(reporter, 'detail', None))
    session_kw.setdefault('confirm', getattr(reporter, 'confirm', None))
    return steps.Session(dev, limits or Limits(), reboot=reboot,
                         export_path=export_path, **session_kw)


def run(session, keys, reporter=None, on_update=None):
    """Run *keys* (in STEPS order) on *session*; progress and each step's
    verdict go to *reporter*.  Returns True when every step passed."""
    todo = run_order(keys)
    done = set()

    def update(key):
        res = session.results.get(key, {})
        # The step's own '=== title (n/N) ===' heading is the session's
        # log; here only its verdict, with the time it took.
        if reporter is not None and res.get('status') != 'running':
            done.add(key)
            reporter.progress(100 * len(done) // max(len(todo), 1))
            sec = res.get('seconds')
            status = res.get('status', '?')
            # A failure's reason is already in the log (the session's
            # "FAILED:" line) and is in the summary: the verdict only.
            line = '{}: {}{}{}'.format(
                title(key), status.upper(),
                '' if status == 'fail' or not res.get('detail') else
                ' -- ' + res['detail'],
                ' ({})'.format(duration(sec)) if sec is not None else '')
            (reporter.warn if status in ('fail', 'warn') else
             reporter.note)(line)
        if on_update is not None:
            on_update(key)

    return session.run(todo, on_update=update)


def save(session, export=False, reporter=None):
    """Save.App + Save.Mfg (and the reboot an i_cont change needs), then optionally
    Export."""
    return run(session, ('save', 'export') if export else ('save',), reporter)


def summary(session, keys):
    """[(n, title, status, detail), ...] for *keys* in run order."""
    out = []
    for i, k in enumerate(run_order(keys), 1):
        res = session.results.get(k, {})
        out.append((i, title(k), res.get('status', 'not run'),
                    res.get('detail', '')))
    return out


def duration(sec):
    """One format for every duration: '41 s', '2 min 05 s'."""
    sec = int(round(sec))
    if sec < 60:
        return '{} s'.format(sec)
    return '{} min {:02d} s'.format(sec // 60, sec % 60)


def _seconds(session, key):
    """duration() for a step that ran, '' otherwise."""
    sec = session.results.get(key, {}).get('seconds')
    return '' if sec is None else duration(sec)


def summary_text(session, keys):
    """The verdict per step, for a dialog in a proportional font as well as
    the log: no column alignment, the detail on its own indented line, one
    duration format, and the total."""
    order = run_order(keys)
    lines = []
    for k, (n, t, s, d) in zip(order, summary(session, keys)):
        sec = _seconds(session, k)
        lines.append('{}. {}: {}{}'.format(
            n, t, s.upper(), ' ({})'.format(sec) if sec else ''))
        if d:
            lines.append('      ' + d)
    total = sum(session.results.get(k, {}).get('seconds') or 0.0
                for k in order)
    if total:
        lines.append('Total: {}'.format(duration(total)))
    return '\n'.join(lines)
