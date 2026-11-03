"""Each step that passes is saved to NV at once (steps.Session._autosave), so
a later failure cannot lose it, and a failed step's RAM never reaches NV."""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..')))

from p4core.actuator import steps                                # noqa: E402


class Node:
    id = 5


class Dev:
    max_temp_c = 80.0

    def __init__(self):
        self.node = Node()

    def _log(self, text):
        pass

    def safe_stop(self):
        pass


def writes(s):
    s.unsaved = True
    return 'ok'


def no_write(s):
    return 'ok'


def warns(s):
    s.unsaved = True
    s.rep.check('something', False, 'a warning')
    return 'ok'


def fails_after_writing(s):
    s.unsaved = True
    raise steps.StepFailed('no good')


def changes_i2t(s):
    s.unsaved = True
    s.ctx['i2t_changed'] = True
    return 'ok'


def table(*pairs):
    return tuple(steps.Step(k, 'G', k, lambda dev: '', fn) for k, fn in pairs)


class AutosaveCase(unittest.TestCase):
    def setUp(self):
        self.events = []
        for target, name, new in (
                (steps.pipeline, 'save_all',
                 lambda dev: self.events.append('save')),
                (steps.commission, 'require_isense', lambda dev: None),
                (steps.commission, 'wait_until_cool',
                 lambda dev, limits, log: None),
                (steps, 'unavailable', lambda node, key: None)):
            p = mock.patch.object(target, name, new)
            p.start()
            self.addCleanup(p.stop)

    def session(self, *pairs, reboot=True, **kw):
        p = mock.patch.object(steps, 'STEPS', table(*pairs))
        p.start()
        self.addCleanup(p.stop)
        return steps.Session(Dev(), reboot=self._reboot if reboot else None,
                             **kw)

    def _reboot(self):
        self.events.append('reboot')


class TestSavedWhenItPasses(AutosaveCase):
    def test_saved_step_by_step(self):
        ran = []

        def second(s):
            ran.append(list(self.events))
            return writes(s)
        s = self.session(('a', writes), ('b', second))
        self.assertTrue(s.run(['a', 'b']))
        self.assertEqual(ran, [['save']])         # before the next step ran
        self.assertEqual(self.events, ['save', 'save'])
        self.assertFalse(s.unsaved)

    def test_a_step_with_nothing_in_ram_costs_no_save(self):
        s = self.session(('a', no_write))
        self.assertTrue(s.run(['a']))
        self.assertEqual(self.events, [])

    def test_a_warning_is_saved_and_stays_a_warning(self):
        s = self.session(('a', warns))
        self.assertTrue(s.run(['a']))
        self.assertEqual(self.events, ['save'])
        self.assertEqual(s.results['a']['status'], 'warn')

    def test_the_save_step_is_not_saved_twice(self):
        s = self.session(('save', steps._r_save), ('export', no_write))
        s.unsaved = True
        self.assertTrue(s.run(['save', 'export']))
        self.assertEqual(self.events, ['save'])

    def test_off_leaves_the_results_in_ram(self):
        s = self.session(('a', writes), autosave=False)
        self.assertTrue(s.run(['a']))
        self.assertEqual(self.events, [])
        self.assertTrue(s.unsaved)


class TestAFailure(AutosaveCase):
    def test_keeps_what_passed_and_saves_nothing_of_its_own(self):
        s = self.session(('a', writes), ('cogging', fails_after_writing),
                         ('c', writes))
        self.assertFalse(s.run(['a', 'cogging', 'c']))
        self.assertEqual(self.events, ['save'])   # a's alone
        self.assertEqual(s.results['a']['status'], 'pass')
        self.assertEqual(s.results['cogging']['status'], 'fail')
        self.assertEqual(s.results['c']['status'], 'skipped')
        self.assertFalse(s.unsaved)               # nothing good is at risk

    def test_the_next_run_reboots_before_it_can_save(self):
        s = self.session(('bad', fails_after_writing), ('a', writes))
        self.assertFalse(s.run(['bad']))
        self.assertEqual(self.events, [])         # left as it failed
        self.assertTrue(s.run(['a']))
        self.assertEqual(self.events, ['reboot', 'save'])
        self.assertTrue(s.run(['a']))             # only once
        self.assertEqual(self.events, ['reboot', 'save', 'save'])

    def test_a_blank_drives_stand_ins_are_set_again_after_that_reboot(self):
        s = self.session(('bad', fails_after_writing), ('a', writes))
        s.ctx['currents_provisional'] = True
        s.run(['bad'])
        s.run(['a'])
        self.assertNotIn('currents_provisional', s.ctx)

    def test_without_a_reboot_the_next_step_is_not_saved(self):
        s = self.session(('bad', fails_after_writing), ('a', writes),
                         reboot=False)
        self.assertFalse(s.run(['bad']))
        self.assertTrue(s.run(['a']))
        self.assertEqual(self.events, [])
        self.assertTrue(s.unsaved)
        self.assertEqual(s.results['a']['status'], 'warn')

    def test_a_reboot_that_fails_does_not_let_the_save_through(self):
        s = self.session(('bad', fails_after_writing), ('a', writes))

        def dead():
            raise ConnectionError('gone')
        s.reboot = dead
        s.run(['bad'])
        self.assertTrue(s.run(['a']))
        self.assertEqual(self.events, [])
        self.assertTrue(s.unsaved)


class TestASaveThatFails(AutosaveCase):
    def failing_save(self):
        def fail(dev):
            raise OSError('no answer')
        return mock.patch.object(steps.pipeline, 'save_all', fail)

    def test_stops_the_run_and_says_so(self):
        s = self.session(('a', writes), ('b', writes))
        with self.failing_save():
            self.assertFalse(s.run(['a', 'b']))
        self.assertEqual(s.results['a']['status'], 'fail')
        self.assertIn('only in RAM', s.results['a']['detail'])
        self.assertEqual(s.results['b']['status'], 'skipped')
        self.assertTrue(s.unsaved)

    def test_keeps_the_good_results_for_a_retry(self):
        s = self.session(('a', writes), ('save', steps._r_save))
        with self.failing_save():
            s.run(['a'])
        self.assertTrue(s.run(['save']))
        self.assertEqual(self.events, ['save'])   # no reboot threw them away
        self.assertFalse(s.unsaved)


class TestI2t(AutosaveCase):
    def test_one_reboot_at_the_end_of_the_run(self):
        s = self.session(('a', changes_i2t), ('b', writes))
        self.assertTrue(s.run(['a', 'b']))
        self.assertEqual(self.events, ['save', 'save', 'reboot'])
        self.assertFalse(s.ctx['i2t_changed'])

    def test_not_after_a_failure(self):
        s = self.session(('a', changes_i2t), ('bad', fails_after_writing))
        self.assertFalse(s.run(['a', 'bad']))
        self.assertEqual(self.events, ['save'])
        self.assertTrue(s.ctx['i2t_changed'])

    def test_a_reboot_that_fails_does_not_fail_the_run(self):
        s = self.session(('a', changes_i2t))

        def dead():
            raise ConnectionError('gone')
        s.reboot = dead
        self.assertTrue(s.run(['a']))
        self.assertTrue(s.ctx['i2t_changed'])


if __name__ == '__main__':
    unittest.main()
