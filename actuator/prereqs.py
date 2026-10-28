"""What each actuator calibration step needs and gives, and the dependency
stages the apps list the steps under.

Every step is declared with:

  requires   resources it cannot run without (an s.need() key, or a
             StepFailed when the drive lacks it);
  uses       resources its result depends on, though it runs without them;
  provides   resources it writes, first time or again;
  handles    for a step that provides a resource an earlier step consumed,
             how that is handled (p4sysid checks the order with it).

A resource is either given before any step (EXTERNAL: the model's config
CSV, the bench supply, the command line) or provided by a step.  A
resource no step in the table provides is taken from the drive (the CSV):
the actuator calibration runs on a model whose identification is done.
p4sysid adds its steps to DEPS.
"""

import collections

#: Given before the first step.  The model's config CSV is loaded first; a
#: drive without it has i_peak 0, and the test current is then 0.
EXTERNAL = {
    'config': 'model CSV loaded: enc_res, v_nom, sense shunt/gain, encoder '
              'model, template i_peak/i_cont/RatedTorque/PWM/filter',
    'bus': 'bench supply at NominalBusVoltage',
    'limits': 'command line: --max-rpm, --max-current, --power, --supply-a',
    'skip_k': 'command line: --skip-k (P4-42: 12), --geared',
    # A template's or a past session's value stands in until its step runs.
    'i_peak': 'template i_peak/i_cont (RatedTorque derived from them: i_peak, or i_cont on fw ecdc33c+)',
    'rated_torque': 'template RatedTorque',
    'pwm': 'template PWM / control rate',
    'budget': 'the drive\'s voltage budget at the template rate',
    'settling_time': 'template MaxSettlingTime',
    'enc_filter': 'template sensor filter',
    'top_speed': '--max-rpm',
}

Dep = collections.namedtuple('Dep', 'requires uses provides handles')


def _d(requires=(), uses=(), provides=(), handles=None):
    return Dep(frozenset(requires), frozenset(uses), frozenset(provides),
               handles or {})


DEPS = {
    'encoder': _d(requires=['config'], provides=['enc_filter']),
    'bias': _d(provides=['isense_bias']),
    'gain': _d(requires=['isense_bias'], uses=['i_cal'],
               provides=['isense_gain']),
    'slope': _d(requires=['isense_bias', 'isense_gain'],
                uses=['settling_time', 'i_cal', 'pwm'],
                provides=['isense_slope']),
    'fold': _d(requires=['isense_slope'], uses=['pwm'],
               provides=['isense_fold']),
    # Winding and cogging drive measured current: the current sense is
    # calibrated first (this also places them when p4sysid is absent).
    'winding': _d(requires=['i_test', 'poles', 'isense_gain'],
                  provides=['e_zero']),
    'runout': _d(requires=['poles', 'e_zero', 'isense_gain'],
                 uses=['i_cal', 'enc_filter', 'skip_k'],
                 provides=['runout', 'enc_filter']),
    'cogging': _d(requires=['poles', 'e_zero', 'vel_gains', 'current_pi',
                            'latency'],
                  uses=['runout', 'enc_filter', 'i_peak'],
                  provides=['cogging']),
    'save': _d(provides=['nv']),
    'export': _d(uses=['nv'],
                 handles={}),
}

#: Steps that only record what the others did: listed last, not staged.
FINAL_GROUP = 'Save'


def stages(order=None, uses=False):
    """{step: (stage, {producer: [resource]})}: stage 1 needs nothing a step
    provides; stage n needs only steps of stages below n.  A resource comes
    from its nearest earlier provider in *order*, as in to_dot(); one the
    step provides itself and that EXTERNAL also gives (r's provisional
    i_peak) is its own.  *uses* stages on the soft edges too.  The Save
    group's steps are left out."""
    from . import steps
    order = list(order or steps.STEP_KEYS)
    group = {st.key: st.group for st in steps.STEPS}
    out = {}
    for i, key in enumerate(order):
        if group.get(key) == FINAL_GROUP:
            continue
        d = DEPS[key]
        src = collections.defaultdict(list)
        for res in sorted(d.requires | (d.uses if uses else frozenset())):
            if res in d.provides and res in EXTERNAL:
                continue
            prod = next((p for p in reversed(order[:i])
                         if res in DEPS[p].provides), None)
            if prod and prod in out:
                src[prod].append(res)
        out[key] = (1 + max((out[p][0] for p in src), default=0), dict(src))
    return out
