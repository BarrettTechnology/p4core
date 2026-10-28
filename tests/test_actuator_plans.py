"""p4core.actuator on its own: the steps the open-source apps run, their
order and numbering, the firmware gate, and that nothing here needs p4sysid.
"""
import os
import subprocess
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, ROOT)
from p4core.actuator import plans, steps                         # noqa: E402
from p4core.tests.fake import current_node, legacy_node          # noqa: E402

# p4sysid registers its steps into the same tables; without it these are
# the actuator steps alone.
ALONE = ('encoder', 'bias', 'gain', 'slope', 'fold', 'winding', 'runout',
         'cogging', 'save', 'export')


def _alone(test):
    """Run the module's checks in a child that cannot import p4sysid."""
    code = ('import sys; sys.path.insert(0, %r); sys.modules["p4sysid"] = None\n'
            'import unittest, p4core.tests.test_actuator_plans as t\n'
            'r = unittest.TextTestRunner(verbosity=0).run(\n'
            '    unittest.defaultTestLoader.loadTestsFromName("Alone.%s", t))\n'
            'sys.exit(not r.wasSuccessful())\n') % (ROOT, test)
    return subprocess.run([sys.executable, '-c', code], capture_output=True,
                          text=True, env=dict(os.environ, P4CORE_ALONE='1'))


class TestStandalone(unittest.TestCase):
    """Each Alone check, in a process where p4sysid cannot be imported."""

    def test_alone(self):
        for name in [n for n in dir(Alone) if n.startswith('test_')]:
            with self.subTest(name):
                r = _alone(name)
                self.assertEqual(r.returncode, 0, r.stderr)

    def test_no_p4sysid_import(self):
        here = os.path.join(ROOT, 'p4core', 'actuator')
        for f in sorted(os.listdir(here)):
            if f.endswith('.py'):
                with open(os.path.join(here, f)) as fh:
                    for line in fh:
                        s = line.strip()
                        self.assertFalse(
                            s.startswith(('import p4sysid', 'from p4sysid')),
                            '{}: {}'.format(f, s))


@unittest.skipUnless(os.environ.get('P4CORE_ALONE'), 'run by TestStandalone')
class Alone(unittest.TestCase):
    def test_steps(self):
        self.assertEqual(steps.STEP_KEYS, ALONE)

    def test_numbered(self):
        # Stage order: the sense first, then what drives measured current.
        got = [(st.key, stage) for _, stage, st in plans.numbered(plans.ACTUATOR)]
        self.assertEqual(got, [('encoder', 1), ('bias', 1), ('gain', 2),
                               ('slope', 3), ('winding', 3), ('fold', 4),
                               ('runout', 4), ('cogging', 4)])

    def test_run_order(self):
        self.assertEqual(plans.run_order(['cogging', 'bias', 'runout']),
                         ['bias', 'runout', 'cogging'])

    def test_aliases(self):
        self.assertEqual(plans.canonical('enczero'), 'winding')
        self.assertEqual(plans.canonical('runout-table'), 'runout')
        with self.assertRaises(plans.PlanError):
            plans.canonical('velocity_refine')

    def test_current_firmware_runs_every_step(self):
        self.assertEqual(plans.available_keys(current_node(), plans.ACTUATOR),
                         plans.ACTUATOR)

    def test_legacy_firmware_runs_the_board_steps(self):
        node = legacy_node()
        self.assertEqual(plans.available_keys(node, plans.ACTUATOR),
                         ('encoder', 'bias', 'gain', 'winding'))
        self.assertIn('needs firmware', plans.unavailable(node, 'cogging'))


class TestRegister(unittest.TestCase):
    def test_unplaced_step_is_refused(self):
        extra = steps.Step('extra', 'Tune', 'Extra', lambda dev: '',
                           lambda s: 'ok')
        with self.assertRaises(ValueError):
            steps.register([extra], steps.STEP_KEYS)


if __name__ == '__main__':
    unittest.main()
