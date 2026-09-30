import contextlib
import io
import math
import os
import tempfile
import threading
import unittest

import fake  # noqa: F401
from fake import FakeNode

from p4core import app_settings, calibrate, cli, ezero, flash, ops, paths
from p4core.reporter import Aborted, ConsoleReporter, RecordingReporter, Reporter


class TestFlashEnums(unittest.TestCase):
    def test_flashloader_protocol_values(self):
        """The flashloader's ProgramCommand numbering; a wrong one erases or
        launches at the wrong moment."""
        F = flash.FlashCommand
        self.assertEqual((F.NO_OPERATION, F.START, F.END, F.ERASE, F.LAUNCH, F.RESET),
                         (0, 1, 2, 3, 4, 5))

    def test_result_values(self):
        R = flash.FlashResult
        self.assertEqual([R.SUCCESS, R.FILE_NOT_FOUND, R.CANDEV_FAILED, R.RESET_FAILED,
                          R.VERSION_INCOMPATIBLE, R.EXTENSION_INVALID, R.ERASE_FAILED,
                          R.FLASH_FAILED, R.CRC_MISMATCH, R.AUTH_FAILED], list(range(10)))
        for r in R:
            self.assertNotEqual(flash.describe(r), 'unknown result')

    def test_version(self):
        self.assertEqual(flash.get_version((4 << 24) | (12 << 8) | 3), '4.12.3')
        node = FakeNode()
        node.values[(0x100A, 0)] = b'5.1.0\x00'
        self.assertEqual(flash.read_version(node), '5.1.0')
        node.values[(0x100A, 0)] = ((2 << 24) | (1 << 8)).to_bytes(4, 'little')
        self.assertEqual(flash.read_version(node), '2.1.0')

    def test_missing_and_bad_files(self):
        self.assertEqual(flash.flash(None, 5, '/nonexistent.bin'),
                         flash.FlashResult.FILE_NOT_FOUND)
        with tempfile.NamedTemporaryFile(suffix='.hex') as f:
            self.assertEqual(flash.flash(None, 5, f.name),
                             flash.FlashResult.EXTENSION_INVALID)


class TestReporter(unittest.TestCase):
    def test_abort_interrupts_sleep(self):
        r = Reporter()
        threading.Timer(0.05, r.abort).start()
        with self.assertRaises(Aborted):
            r.sleep(5)

    def test_console_confirm_not_interactive(self):
        out = io.StringIO()
        r = ConsoleReporter(stream=out, interactive=False)
        self.assertFalse(r.confirm('go?'))
        self.assertTrue(r.confirm('go?', default=True))
        self.assertTrue(ConsoleReporter(stream=out, assume_yes=True).confirm('go?'))

    def test_console_progress_to_a_pipe(self):
        out = io.StringIO()
        r = ConsoleReporter(stream=out, interactive=False)
        for p in range(101):
            r.progress(p)
        self.assertEqual(out.getvalue().count('%'), 11)


class TestPaths(unittest.TestCase):
    def test_data(self):
        self.assertTrue(os.path.isfile(paths.puck4_eds()))
        self.assertTrue(os.path.isfile(paths.flashloader_eds()))
        self.assertIn('P4-42-Blank.csv', paths.templates())

    def test_settings(self):
        with tempfile.TemporaryDirectory() as d:
            s = app_settings.AppSettings('x', directory=d)
            s['port'] = 'can1'
            self.assertEqual(app_settings.AppSettings('x', directory=d).get('port'), 'can1')
            with open(os.path.join(d, 'settings.json'), 'w') as f:
                f.write('{corrupt')
            self.assertEqual(app_settings.AppSettings('x', directory=d).as_dict(), {})


class SimPuck(FakeNode):
    """Enough of a puck for the calibrations: the d-axis current follows ud,
    the rotor follows the field angle, the current sense sees both."""

    RES = 4096
    POLES = 14
    E_ZERO = 1234 % (4096 // 7)
    POLARITY = -1
    GAIN_RATIO = 1.02          # beta reads 2 % low

    def __init__(self, bias=(2048 * 16, 2050 * 16), stuck=False):
        super().__init__(5)
        self.stuck = stuck
        self.bias = bias
        for name, sub, value in [
                ('MfgSoftwareVersion', None, (5 << 24)),
                ('EncoderConfig', 'Resolution', self.RES),
                ('Calibration', 'poles', self.POLES),
                ('Calibration', 'i_peak', 10000),
                ('Calibration', 'i_cal', 1000),
                ('Calibration', 'e_zero', 0),
                ('Calibration', 'e_polarity', 1),
                ('Alpha', 'Gainfactor', 4096), ('Beta', 'Gainfactor', 4096),
                ('Alpha', 'Bias', 0), ('Beta', 'Bias', 0),
                ('Motor', 'ud', 0), ('Motor', 'uq', 0), ('Theta_e', None, 0),
                ('SetModeOfOperation', None, 0), ('ControlWord', None, 0),
                ('Save', 'Single', 0), ('PositionFeedback', None, 100)]:
            self.set(name, sub, value)
        self.hook_read('Motor', 'id', self._id)
        self.hook_read('Encoder', 'RawPosition', self._raw)
        self.hook_read('Alpha', 'Filtered', lambda: self._sense(math.cos, 0))
        self.hook_read('Beta', 'Filtered', lambda: self._sense(math.sin, 1))
        self.turns = 0.0
        self.last_theta = 0

    def _ud(self):
        return self.get('Motor', 'ud') if self.get('SetModeOfOperation') == 12 else 0

    def _id(self):
        # per-mille of i_peak: 1 A at ud 1000
        return min(self._ud() / 10, 1000)

    def _theta(self):
        return self.get('Theta_e') % 65536

    def _raw(self):
        if self.stuck:
            return 77
        theta = self._theta()
        step = (theta - self.last_theta + 32768) % 65536 - 32768
        self.last_theta = theta
        self.turns += step / 65536.0
        cpe = self.RES / (self.POLES // 2)
        return int(round(self.E_ZERO + self.POLARITY * self.turns * cpe)) % self.RES

    def _sense(self, fn, ch):
        amp = self._id() / 1000.0 * 10000 / 1000 * 800   # 800 Q12.4 counts per A
        if ch:
            amp /= self.GAIN_RATIO
        return int(self.bias[ch] + amp * fn(2 * math.pi * self._theta() / 65536))


def quick(r):
    r.sleep = lambda s: None
    return r


class TestCalibrate(unittest.TestCase):
    def test_bias(self):
        puck = SimPuck()
        res = calibrate.calibrate_bias(puck, quick(RecordingReporter()))
        self.assertTrue(res['ok'])
        self.assertEqual(puck.get('Alpha', 'Bias'), 2048 * 16)
        self.assertIn(((0x1010 + 0, 4), (0x3009 << 8) | 3),
                      [(k, v) for k, v in puck.writes if k == puck.key('Save', 'Single')])
        self.assertEqual(puck.get('SetModeOfOperation'), 0)

    def test_bias_out_of_bounds_declined(self):
        puck = SimPuck(bias=(2300 * 16, 2048 * 16))
        with self.assertRaises(calibrate.CalibrationError):
            calibrate.calibrate_bias(puck, quick(RecordingReporter(answers=[False])))

    def test_gain(self):
        puck = SimPuck()
        res = calibrate.calibrate_gain(puck, quick(RecordingReporter()))
        self.assertAlmostEqual(res['beta_gain'] / 4096.0, SimPuck.GAIN_RATIO, places=2)
        self.assertEqual(puck.get('Beta', 'Gainfactor'), res['beta_gain'])
        self.assertEqual(puck.get('Motor', 'ud'), 0)

    def test_gain_no_current_restores(self):
        puck = SimPuck()
        puck.hook_read('Motor', 'id', lambda: 0)
        puck.set('Beta', 'Gainfactor', 4000)
        with self.assertRaises(calibrate.CalibrationError):
            calibrate.calibrate_gain(puck, quick(RecordingReporter()))
        self.assertEqual(puck.get('Beta', 'Gainfactor'), 4000)

    def test_ezero(self):
        puck = SimPuck()
        rep = quick(RecordingReporter())
        res = calibrate.calibrate_ezero(puck, rep)
        self.assertEqual(res['e_polarity'], SimPuck.POLARITY)
        self.assertEqual(puck.get('Calibration', 'e_zero'), SimPuck.E_ZERO)
        self.assertEqual(puck.get('SetModeOfOperation'), 0)

    def test_ezero_stalled_stores_nothing(self):
        puck = SimPuck(stuck=True)
        with self.assertRaises(calibrate.CalibrationError):
            calibrate.calibrate_ezero(puck, quick(RecordingReporter()))
        self.assertEqual(puck.get('Calibration', 'e_zero'), 0)

    def test_all_in_order(self):
        puck = SimPuck()
        rep = quick(RecordingReporter())
        res = calibrate.calibrate_all(puck, rep, settle_s=0)
        self.assertEqual(list(res), ['encoder-test', 'bias', 'gain', 'ezero'])

    def test_ramp_applies_before_measuring(self):
        """A stale id reading from the previous step must not end the ramp at ud 0."""
        puck = SimPuck()
        puck.hook_read('Motor', 'id', lambda: 1000 if puck.get('Motor', 'ud') == 0 else 0)
        self.assertGreater(calibrate.ramp_to_calibration_current(puck, quick(Reporter())), 0)


class FakeSession:
    BOOT_S = 0
    REBOOT_TIMEOUT_S = 0.2

    def __init__(self, nodes):
        self.nodes = {n.id: n for n in nodes}
        self.found_ids = sorted(self.nodes)
        self.node = None
        self.port = 'fake0'
        self.nmt = []

    def scan(self):
        self.found_ids = sorted(self.nodes)
        return self.found_ids

    def add_node(self, node_id):
        if node_id not in self.nodes:
            raise KeyError(node_id)
        return self.nodes[node_id]

    def _forget(self, node_id):
        pass

    def send_nmt(self, command, node_id):
        self.nmt.append((command, node_id))
        node = self.nodes.pop(node_id)
        node.id = node.values[node.key('NetCfg')]
        self.nodes[node.id] = node

    def set_sync_rate(self, hz):
        pass

    def select_node(self, node_id):
        self.node = self.nodes[node_id]
        return self.node

    def connect(self, port, fd=None):
        self.port = port

    def disconnect(self):
        pass


class TestOps(unittest.TestCase):
    def puck(self, node_id):
        node = FakeNode(node_id)
        node.set('NetCfg', None, node_id)
        node.values[(0x1000, 0)] = 0x20192
        for index, base in ops.config_csv.PDO_COB_BASES.items():
            node.values[(index, 1)] = base | node_id
        return node

    def test_set_id(self):
        session = FakeSession([self.puck(5)])
        rep = quick(RecordingReporter())
        self.assertTrue(ops.set_id(session, 5, 9, rep))
        node = session.nodes[9]
        self.assertEqual(node.values[(0x1800, 1)], 0x189)
        self.assertEqual(session.nmt, [(0x81, 5)])
        self.assertEqual(session.found_ids, [9])

    def test_set_id_refuses_taken(self):
        session = FakeSession([self.puck(5), self.puck(9)])
        with self.assertRaises(ops.OperationError):
            ops.set_id(session, 5, 9)

    def test_flash_refuses_golden(self):
        with self.assertRaises(ops.OperationError):
            ops.flash(FakeSession([]), 5, '/fw/P4-42-GOLDEN.bin')


class TestCli(unittest.TestCase):
    def parser(self):
        parser, sub = cli.make_parser('t', 'test')
        cli.add_common_commands(sub)
        return parser

    def test_options_before_or_after_the_command(self):
        p = self.parser()
        a = p.parse_args(['--can', 'can3', 'info', '--id', '5'])
        self.assertEqual((a.can, a.node_id), ('can3', 5))
        a = p.parse_args(['--id', '5', 'config', 'load', 'x.csv', '--no-save'])
        self.assertEqual((a.node_id, a.csv, a.no_save), (5, 'x.csv', True))

    def test_main_runs_set_id(self):
        session = FakeSession([TestOps().puck(5)])
        code = cli.main(self.parser(), ['-q', 'set-id', '12'],
                        session_factory=lambda: session)
        self.assertEqual(code, cli.EXIT_OK)
        self.assertIn(12, session.nodes)

    def test_main_needs_id_with_two_pucks(self):
        ops_ = TestOps()
        session = FakeSession([ops_.puck(5), ops_.puck(6)])
        code = cli.main(self.parser(), ['reset'], session_factory=lambda: session)
        self.assertEqual(code, cli.EXIT_NO_DEVICE)

    def test_config_check_needs_no_bus(self):
        tmpl = paths.template('P4-42-Blank.csv')
        self.assertEqual(cli.main(self.parser(), ['-q', 'config', 'check', tmpl],
                                  session_factory=None), cli.EXIT_OK)

    def test_calibrate_rejects_unknown_step(self):
        session = FakeSession([TestOps().puck(5)])
        code = cli.main(self.parser(), ['-y', 'calibrate', 'bogus'],
                        session_factory=lambda: session)
        self.assertEqual(code, cli.EXIT_USAGE)

    def test_an_sdo_abort_is_a_failure_not_a_traceback(self):
        from canopen.sdo.exceptions import SdoAbortedError

        def refuse(ctx):
            raise SdoAbortedError(0x08000022)

        parser, sub = cli.make_parser('t', 'test')
        p = sub.add_parser('boom')
        p.set_defaults(func=refuse)
        session = FakeSession([TestOps().puck(5)])
        err = io.StringIO()
        with contextlib.redirect_stderr(err), contextlib.redirect_stdout(err):
            code = cli.main(parser, ['boom'], session_factory=lambda: session)
        self.assertEqual(code, cli.EXIT_FAILED)
        self.assertIn('0x08000022', err.getvalue())


if __name__ == '__main__':
    unittest.main()
