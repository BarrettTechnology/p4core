"""worker: the GUI worker-process children (flash, config load)."""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(__file__))

import fake  # noqa: E402,F401  (puts p4core on the path)

from p4core import config_csv, worker                          # noqa: E402
from p4core import flash as _flash                             # noqa: E402


class Queue(list):
    put = list.append


class TestDeviceChildren(unittest.TestCase):

    def setUp(self):
        patcher = mock.patch('builtins.print')      # the children log to stdout
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_flash_refuses_golden_and_says_fail(self):
        q = Queue()
        with mock.patch.object(_flash, 'flash_port') as flash_port:
            worker.flash_child('can0', 5, '/x/P4-GOLDEN-4.4.0.bin', q)
        flash_port.assert_not_called()
        self.assertEqual(q, ['Fail'])

    def test_flash_pass(self):
        q = Queue()

        def fake(port, node_id, path, reporter):
            reporter.progress(0)
            reporter.progress(50)
            reporter.progress(50)          # repeats are not queued
            reporter.progress(100)
            return _flash.FlashResult.SUCCESS

        with mock.patch.object(_flash, 'flash_port', side_effect=fake):
            worker.flash_child('can0', 5, 'app.bin', q)
        self.assertEqual(q, [0, 50, 100, 'Pass'])

    def test_flash_failure_and_exception_say_fail(self):
        for effect in (_flash.FlashResult.CRC_MISMATCH, OSError('bus')):
            q = Queue()
            with mock.patch.object(_flash, 'flash_port', side_effect=[effect]
                                   if isinstance(effect, Exception) else None,
                                   return_value=effect), \
                    mock.patch('traceback.print_exc'):
                worker.flash_child('can0', 5, 'app.bin', q)
            self.assertEqual(q[-1], 'Fail')

    def _config(self, result=None, error=None):
        q = Queue()
        network = mock.Mock()
        load = mock.Mock(return_value=result, side_effect=error)
        with mock.patch('p4core.can_backend.make_network',
                        return_value=network), \
                mock.patch.object(config_csv, 'load', load):
            worker.config_child('can0', 7, 'm.csv', q)
        network.disconnect.assert_called_once()
        return q, load

    def test_config_writes_only(self):
        result = config_csv.ApplyResult()
        result.written = 3
        q, load = self._config(result)
        self.assertEqual(q, ['Pass'])
        self.assertEqual(load.call_args.args[1:3], ('m.csv', 7))

    def test_config_errors_or_refusal_say_fail(self):
        result = config_csv.ApplyResult()
        result.errors = ['line 3: bad']
        self.assertEqual(self._config(result)[0], ['Fail'])
        self.assertEqual(self._config(error=config_csv.ConfigError(
            []))[0], ['Fail'])


if __name__ == '__main__':
    unittest.main()
