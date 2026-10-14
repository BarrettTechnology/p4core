"""reporter.py — how an operation talks to whoever started it.

Every long operation in p4core and in the apps takes a Reporter instead of
reaching for a status bar, a progress gauge, a dialog or ``wx.Yield``.  The
command line hands it a ConsoleReporter; the GUI hands it a WxReporter
(in the app's gui package), which marshals each call onto the GUI thread.  The
operation neither knows nor cares which.

The interface is small on purpose:

  status(text)          one line of what is happening now
  progress(pct, text)   0-100, for a gauge; text is optional
  note(text)            something worth keeping in the log / scrollback
  warn(text)            something the user should look at
  confirm(question)     a yes/no the operation cannot decide by itself
  check_abort()         raise Aborted if the user asked to stop
  sleep(seconds)        a wait that stays abortable

``confirm`` answers the default without asking when the reporter is not
interactive (a script, a CI run): an operation must never hang waiting for a
keyboard that is not there.
"""

import sys
import threading
import time


class Aborted(Exception):
    """The user asked the operation to stop."""


class Reporter:
    """Base reporter: silent, never interactive, never aborts.

    Subclasses override what they can show.  ``abort()`` may be called from
    any thread; the running operation notices at its next ``check_abort()``
    or ``sleep()``.
    """

    #: Whether confirm() may ask a person.  When False it returns its default.
    interactive = False

    #: How often sleep() polls the abort flag (s).
    SLEEP_SLICE = 0.02

    def __init__(self):
        self._abort = threading.Event()

    # -- output --------------------------------------------------------
    def status(self, text):
        pass

    def progress(self, pct, text=None):
        pass

    def note(self, text):
        pass

    def warn(self, text):
        self.note('WARNING: ' + text)

    # -- input ---------------------------------------------------------
    def confirm(self, question, default=False):
        return default

    # -- cancellation --------------------------------------------------
    def abort(self):
        self._abort.set()

    def reset_abort(self):
        self._abort.clear()

    @property
    def aborted(self):
        return self._abort.is_set()

    def check_abort(self):
        if self._abort.is_set():
            raise Aborted('stopped by request')

    def sleep(self, seconds):
        """Wait *seconds*, raising Aborted as soon as an abort is requested."""
        end = time.monotonic() + seconds
        while True:
            self.check_abort()
            left = end - time.monotonic()
            if left <= 0:
                return
            self._abort.wait(min(left, self.SLEEP_SLICE))

    # -- adapters for older call signatures ----------------------------
    def log(self, text):
        """``log=`` callback style (p4sysid, remap_pdo_cob_ids)."""
        self.note(text)

    def should_abort(self):
        """``should_abort=`` callback style (p4sysid)."""
        return self.aborted


NullReporter = Reporter


class ConsoleReporter(Reporter):
    """Reporter for a terminal: status and notes to stdout, warnings to
    stderr, a single-line progress bar when stdout is a TTY, and y/n
    questions on stdin when *interactive* (default: stdin is a TTY).

    *assume_yes* answers every confirm() with yes (the CLI's ``--yes``).
    *quiet* drops status lines, keeping notes, warnings and errors.
    """

    def __init__(self, stream=None, interactive=None, assume_yes=False,
                 quiet=False):
        super().__init__()
        self.stream = stream or sys.stdout
        if interactive is None:
            interactive = bool(getattr(sys.stdin, 'isatty', lambda: False)())
        self.interactive = interactive
        self.assume_yes = assume_yes
        self.quiet = quiet
        self._bar_open = False
        self._last_pct = None
        self._tty = bool(getattr(self.stream, 'isatty', lambda: False)())

    def _end_bar(self):
        if self._bar_open:
            self.stream.write('\n')
            self._bar_open = False
            self._last_pct = None

    def _line(self, text, stream=None):
        self._end_bar()
        stream = stream or self.stream
        stream.write(text + '\n')
        stream.flush()

    def status(self, text):
        if not self.quiet:
            self._line(text)

    def note(self, text):
        self._line(text)

    def warn(self, text):
        self._line('WARNING: ' + text, sys.stderr)

    def progress(self, pct, text=None):
        pct = max(0, min(100, int(pct)))
        if self._tty:
            self.stream.write('\r[{:<50}] {:3d}%{}'.format(
                '=' * (pct // 2), pct, ' ' + text if text else ''))
            self.stream.flush()
            self._bar_open = pct < 100
            if pct >= 100:
                self.stream.write('\n')
        elif pct != self._last_pct and (pct % 10 == 0 or pct >= 100):
            # Not a terminal (a log, a pipe): every 10 % is plenty.
            self._line('{:3d}%{}'.format(pct, ' ' + text if text else ''))
        self._last_pct = pct

    def confirm(self, question, default=False):
        if self.assume_yes:
            self._line(question + ' [yes: --yes]')
            return True
        if not self.interactive:
            self._line('{} [{}: not interactive]'.format(
                question, 'yes' if default else 'no'))
            return default
        self._end_bar()
        hint = '[Y/n]' if default else '[y/N]'
        while True:
            self.stream.write('{} {} '.format(question, hint))
            self.stream.flush()
            answer = sys.stdin.readline()
            if not answer:                      # EOF
                return default
            answer = answer.strip().lower()
            if not answer:
                return default
            if answer in ('y', 'yes'):
                return True
            if answer in ('n', 'no'):
                return False


class RecordingReporter(Reporter):
    """Keeps every call, for tests.  *answers* feeds confirm() in order."""

    def __init__(self, answers=()):
        super().__init__()
        self.events = []
        self.answers = list(answers)

    def status(self, text):
        self.events.append(('status', text))

    def progress(self, pct, text=None):
        self.events.append(('progress', pct))

    def note(self, text):
        self.events.append(('note', text))

    def warn(self, text):
        self.events.append(('warn', text))

    def confirm(self, question, default=False):
        self.events.append(('confirm', question))
        return self.answers.pop(0) if self.answers else default

    def texts(self, kind=None):
        return [t for k, t in self.events if kind is None or k == kind]
