"""wx_reporter.py — a Reporter for wxPython frames.

Operations run on a worker thread; every call here is marshalled onto the
GUI thread with wx.CallAfter.  confirm() blocks the worker until the user
answers the dialog (it must never be called from the GUI thread itself —
the operation would deadlock waiting for an event loop it is blocking).

    reporter = WxReporter(frame, statusbar=frame.frame_statusbar,
                          gauge=frame.gauge, log=print)
    threading.Thread(target=lambda: ops.flash(session, 5, path, reporter)).start()
    ...
    stop_button -> reporter.abort()
"""

import threading

import wx

from .reporter import Reporter


class WxReporter(Reporter):
    interactive = True

    def __init__(self, parent, statusbar=None, gauge=None, log=None,
                 title='P4'):
        super().__init__()
        self.parent = parent
        self.statusbar = statusbar
        self.gauge = gauge
        self.log_fn = log
        self.title = title

    @staticmethod
    def _alive(window):
        return window is not None and bool(window)

    def status(self, text):
        if self._alive(self.statusbar):
            wx.CallAfter(lambda: self._alive(self.statusbar)
                         and self.statusbar.SetStatusText(text))
        if self.log_fn:
            self.log_fn(text)

    def progress(self, pct, text=None):
        if self._alive(self.gauge):
            value = max(0, min(100, int(pct)))
            wx.CallAfter(lambda: self._alive(self.gauge)
                         and self.gauge.SetValue(value))
        if text:
            self.status(text)

    def note(self, text):
        if self.log_fn:
            self.log_fn(text)

    def warn(self, text):
        self.note('WARNING: ' + text)
        if self._alive(self.statusbar):
            wx.CallAfter(lambda: self._alive(self.statusbar)
                         and self.statusbar.SetStatusText(text))

    def confirm(self, question, default=False):
        if wx.IsMainThread():
            return self._ask(question, default)
        answer = [default]
        done = threading.Event()

        def ask():
            try:
                answer[0] = self._ask(question, default)
            finally:
                done.set()

        wx.CallAfter(ask)
        while not done.wait(0.1):
            if self.aborted:
                return default
        return answer[0]

    def _ask(self, question, default):
        style = wx.YES_NO | wx.ICON_QUESTION
        if not default:
            style |= wx.NO_DEFAULT
        dlg = wx.MessageDialog(self.parent if self._alive(self.parent) else None,
                               question, self.title, style)
        try:
            return dlg.ShowModal() == wx.ID_YES
        finally:
            dlg.Destroy()
