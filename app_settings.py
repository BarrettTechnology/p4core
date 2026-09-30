"""app_settings.py — small persistent per-user settings (JSON).

    settings = AppSettings('pucktuner')
    port = settings.get('can_port', 'can0')
    settings['can_port'] = 'can1'      # saved immediately

Stored in paths.user_config_dir(app)/settings.json.  A missing or corrupt
file reads as empty; a failed write is ignored (settings are a convenience,
never a reason to stop).
"""

import json
import os
import threading

from . import paths


class AppSettings:
    FILENAME = 'settings.json'

    def __init__(self, app, directory=None):
        self.app = app
        self.path = os.path.join(directory or paths.user_config_dir(app),
                                 self.FILENAME)
        self._lock = threading.Lock()
        self._data = self._read()

    def _read(self):
        try:
            with open(self.path) as f:
                data = json.load(f)
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def save(self):
        with self._lock:
            tmp = self.path + '.tmp'
            try:
                with open(tmp, 'w') as f:
                    json.dump(self._data, f, indent=2, sort_keys=True)
                os.replace(tmp, self.path)
            except OSError:
                pass

    def get(self, key, default=None):
        return self._data.get(key, default)

    def set(self, key, value):
        self._data[key] = value
        self.save()

    def update(self, **values):
        self._data.update(values)
        self.save()

    def __getitem__(self, key):
        return self._data[key]

    def __setitem__(self, key, value):
        self.set(key, value)

    def __contains__(self, key):
        return key in self._data

    def as_dict(self):
        return dict(self._data)
