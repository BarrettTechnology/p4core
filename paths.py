"""paths.py — where p4core's and the apps' files are, however they are run.

Three ways to run, and each puts files somewhere else:

  from a checkout   data lives beside the package (``p4core/data``)
  pip-installed     the same, inside site-packages
  PyInstaller       one-file builds unpack to ``sys._MEIPASS``; one-dir
                    builds keep data beside the executable

Every lookup goes through here, so nothing opens ``'puck4.eds'`` relative to
whatever the working directory happens to be (the old apps did, and had to
``os.chdir`` to the executable's directory to get away with it).

Writable files never go beside the code: a pip install is usually read-only.
They go to the per-user directories below.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(_HERE, 'data')


def frozen():
    """True in a PyInstaller build."""
    return bool(getattr(sys, 'frozen', False))


def resource(package_dir, *parts):
    """A read-only file shipped with a package.

    *package_dir* is the package's own directory (pass
    ``os.path.dirname(__file__)``).  In a frozen build the file is looked for
    under ``sys._MEIPASS`` and beside the executable too, with the package's
    name as the first path component and without it, which covers both the
    ``--add-data pkg/data:pkg/data`` and the copy-beside-the-binary layouts
    the build scripts use.  Returns the first path that exists, or the
    in-package path if none does (so the error names the expected place).
    """
    inside = os.path.join(package_dir, *parts)
    if os.path.exists(inside) or not frozen():
        return inside
    name = os.path.basename(package_dir.rstrip(os.sep))
    roots = [getattr(sys, '_MEIPASS', None),
             os.path.dirname(os.path.abspath(sys.executable))]
    for root in filter(None, roots):
        for candidate in (os.path.join(root, name, *parts),
                          os.path.join(root, *parts)):
            if os.path.exists(candidate):
                return candidate
    return inside


def data(*parts):
    """A file from p4core's own ``data`` directory."""
    return resource(_HERE, 'data', *parts)


def puck4_eds():
    """The application object dictionary, shared by every app."""
    return data('puck4.eds')


def flashloader_eds():
    return data('flashloader.eds')


def template(name):
    """A blank config CSV template, e.g. ``template('P4-42-Blank.csv')``."""
    return data('templates', name)


def templates():
    """Names of every shipped config template."""
    folder = data('templates')
    try:
        return sorted(f for f in os.listdir(folder) if f.endswith('.csv'))
    except OSError:
        return []


def _xdg(var, fallback):
    base = os.environ.get(var)
    if base:
        return base
    return os.path.join(os.path.expanduser('~'), fallback)


def user_config_dir(app):
    """Per-user settings directory for *app*, created on demand."""
    if sys.platform == 'win32':
        base = os.environ.get('APPDATA') or os.path.expanduser('~')
    elif sys.platform == 'darwin':
        base = os.path.expanduser('~/Library/Application Support')
    else:
        base = _xdg('XDG_CONFIG_HOME', '.config')
    path = os.path.join(base, app)
    os.makedirs(path, exist_ok=True)
    return path


def user_data_dir(app):
    """Per-user data directory for *app* (logs, captures), created on demand."""
    if sys.platform == 'win32':
        base = os.environ.get('LOCALAPPDATA') or os.path.expanduser('~')
    elif sys.platform == 'darwin':
        base = os.path.expanduser('~/Library/Application Support')
    else:
        base = _xdg('XDG_DATA_HOME', os.path.join('.local', 'share'))
    path = os.path.join(base, app)
    os.makedirs(path, exist_ok=True)
    return path
