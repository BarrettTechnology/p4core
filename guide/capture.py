"""Screenshot capture, run INSIDE the app's own Python (its venv, its source on PYTHONPATH).

    python -c "import p4core.guide.capture as c; c.run('spec.json')"

The one p4core module that uses wx: it is imported only inside run(), in the app's process
(p4core.guide.shots starts it), so `import p4core` still never pulls in wx.

spec.json (written by p4core.guide.shots): {"main": "puckutility.gui.app:main", "args": [...],
"out": DIR, "startup_s": 3, "shots": [{"name": ..., "steps": [...]}, ...]}

Each shot's steps run in order on the GUI thread, then the app's main window is captured:
    wait 2.5                    let the app work (the event loop keeps running)
    any other line              Python, run with f = the main frame, wx, app = wx.GetApp()
                                (e.g.  f.scan_pucks(None)   f.text_version.SetValue('5.0.5'))
Writes DIR/<name>.png (the client area) and DIR/<name>.json: the window title, client size,
every named widget's rectangle (f.<attr>, client pixels), the menu bar (menus and their items)
and the status bar text. Modal dialogs never block a capture: they are logged and answered OK.
"""
import importlib
import json
import os
import sys
import time
import traceback

wx = None          # imported by run(), in the app's own process
TARGET_PX = 2400   # saved width: about 2400 px whatever the window size (>= 400 ppi printed)


def _log(*a):
    print('[capture]', *a, flush=True)


def _rects(f):
    org = f.ClientToScreen((0, 0))
    out = {}
    named = list(vars(f).items())
    for name, obj in list(named):           # helper objects holding widgets: f.can_bar.btn_scan
        if not isinstance(obj, wx.Window) and hasattr(obj, '__dict__') and not isinstance(obj, type):
            try:
                named += [(name + '.' + k, v) for k, v in vars(obj).items() if isinstance(v, wx.Window)]
            except TypeError:
                pass
    for name, w in named:
        try:
            if isinstance(w, wx.Window) and w.IsShownOnScreen():
                r = w.GetScreenRect()
                out[name] = [r.x - org[0], r.y - org[1], r.width, r.height]
        except Exception:
            pass
    return out


def _children(w):
    lst = w.GetChildren()
    return [lst[i] for i in range(len(lst))]     # (wxPython 4.2.2 / Python 3.13: no iterator)


def _labelled(f, out):
    """Every visible control with on-screen text, as 'text:<label>' (first one wins), plus each
    notebook tab as 'tab:<label>' (its whole notebook's rectangle; the tabs are not windows)."""
    org = f.ClientToScreen((0, 0))
    stack = _children(f)
    while stack:
        w = stack.pop(0)
        try:
            if not w.IsShownOnScreen():
                continue
            r = w.GetScreenRect()
            rect = [r.x - org[0], r.y - org[1], r.width, r.height]
            label = w.GetLabel().replace('&', '').strip() if hasattr(w, 'GetLabel') else ''
            if isinstance(w, (wx.Choice, wx.ComboBox)):
                label = ''
            if label and 'text:' + label not in out:
                out['text:' + label] = rect
            if isinstance(w, wx.Notebook):
                for i in range(w.GetPageCount()):
                    out.setdefault('tab:' + w.GetPageText(i), rect)
            stack.extend(_children(w))
        except Exception:
            pass
    return out


def _menus(f):
    mb = f.GetMenuBar()
    if mb is None:
        return []

    def items(menu):
        out = []
        for it in menu.GetMenuItems():
            if it.IsSeparator():
                out.append({'sep': True})
                continue
            d = {'label': it.GetItemLabelText(), 'accel': (it.GetItemLabel().split('\t') + [''])[1]}
            if it.GetSubMenu() is not None:
                d['items'] = items(it.GetSubMenu())
            if it.IsCheckable():
                d['checked'] = it.IsChecked()
            out.append(d)
        return out
    return [{'label': mb.GetMenuLabelText(i), 'items': items(mb.GetMenu(i))} for i in range(mb.GetMenuCount())]


def _status(f):
    sb = f.GetStatusBar()
    if sb is None:
        return []
    return [sb.GetStatusText(i) for i in range(sb.GetFieldsCount())]


def _grab(f, path):
    # GTK draws a window without focus in its faded 'backdrop' style: raise and focus it first
    f.Raise()
    f.SetFocus()
    for _ in range(10):
        wx.SafeYield()
        if f.IsActive():
            break
        time.sleep(0.05)
    if not f.IsActive():
        _log('warning: the window is not active (another window has focus): the capture may look faded')
    f.Update()
    wx.SafeYield()
    w, h = f.GetClientSize()
    bmp = wx.Bitmap(w, h)
    mdc = wx.MemoryDC(bmp)
    mdc.Blit(0, 0, w, h, wx.ClientDC(f), 0, 0)
    mdc.SelectObject(wx.NullBitmap)
    # Saved a whole factor larger, nearest-neighbour (each pixel a sharp block, nothing blurred): printed at
    # ~5-7 in, a 1:1 capture is ~100-200 ppi and PDF viewers smooth it; this keeps text crisp. The
    # .json keeps client pixels, so callout positions do not depend on it.
    img = bmp.ConvertToImage()
    k = max(2, -(-TARGET_PX // w))            # a whole factor, so every pixel stays a square block
    img = img.Scale(w * k, h * k, wx.IMAGE_QUALITY_NEAREST)
    img.SaveFile(path, wx.BITMAP_TYPE_PNG)
    return w, h


def run(spec_path):
    global wx
    import wx as _wx
    wx = _wx
    spec = json.load(open(spec_path))
    out = spec['out']
    os.makedirs(out, exist_ok=True)
    shots = list(spec['shots'])
    state = {'armed': False, 'failed': []}

    # never block on a modal dialog while capturing
    def _modal(self):
        _log('dialog answered OK:', type(self).__name__, repr(self.GetTitle()))
        return wx.ID_OK
    wx.Dialog.ShowModal = _modal
    wx.MessageDialog.ShowModal = _modal
    wx.MessageBox = lambda *a, **k: (_log('message box:', a[:1]), wx.OK)[1]

    def frame():
        return wx.GetApp().GetTopWindow()

    def finish():
        _log('done' if not state['failed'] else 'done with errors: ' + ', '.join(state['failed']))
        sys.stdout.flush()
        os._exit(1 if state['failed'] else 0)

    def do_shot(i=0):
        if i >= len(shots):
            finish()
            return
        shot = shots[i]
        steps = list(shot.get('steps', []))
        f = frame()
        env = {'f': f, 'wx': wx, 'app': wx.GetApp(), 'time': time}

        def step(k=0):
            if k >= len(steps):
                try:
                    png = os.path.join(out, shot['name'] + '.png')
                    w, h = _grab(f, png)
                    json.dump({'name': shot['name'], 'title': f.GetTitle(), 'size': [w, h],
                               'widgets': _labelled(f, _rects(f)), 'menus': _menus(f), 'status': _status(f)},
                              open(os.path.join(out, shot['name'] + '.json'), 'w'), indent=1)
                    _log('captured', shot['name'], '{}x{}'.format(w, h))
                except Exception:
                    traceback.print_exc()
                    state['failed'].append(shot['name'])
                wx.CallLater(300, do_shot, i + 1)
                return
            line = steps[k].strip()
            if not line or line.startswith('#'):
                step(k + 1)
                return
            if line.split()[0] == 'wait':
                wx.CallLater(int(1000 * float(line.split()[1])), step, k + 1)
                return
            try:
                exec(line, env)
            except Exception:
                _log('step failed in {}: {}'.format(shot['name'], line))
                traceback.print_exc()
                state['failed'].append(shot['name'])
            wx.CallLater(50, step, k + 1)
        step()

    orig_show = wx.Frame.Show

    def show(self, show=True):
        if not state['armed']:
            state['armed'] = True
            wx.CallLater(int(1000 * spec.get('startup_s', 3)), do_shot)
            wx.CallLater(int(1000 * spec.get('timeout_s', 300)), lambda: (_log('TIMEOUT'), finish()))
        return orig_show(self, show)
    wx.Frame.Show = show

    mod, fn = spec['main'].split(':')
    sys.argv = [mod] + list(spec.get('args', []))
    getattr(importlib.import_module(mod), fn)(list(spec.get('args', [])))
