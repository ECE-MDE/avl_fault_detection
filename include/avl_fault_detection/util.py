from __future__ import annotations
import os
from pathlib import Path
from typing import Tuple


def get_newest_subdir(path: Path):
    all_subdirs = [d for d in os.listdir(path.stem) if os.path.isdir(d)]
    return max(all_subdirs, key=os.path.getmtime)

# https://stackoverflow.com/questions/5389507/iterating-over-every-two-elements-in-a-list
def grouped(iterable, n):
    "s -> (s0,s1,s2,...sn-1), (sn,sn+1,sn+2,...s2n-1), (s2n,s2n+1,s2n+2,...s3n-1), ..."
    return zip(*[iter(iterable)]*n)

class Flag:
    def __init__(self, state: bool = False):
        self._state = state
    
    def toggle(self) -> bool:
        self._state = not self._state
        return self._state

    def set(self, state: bool) -> bool:
        self._state = state
        return self._state
    
    def get(self) -> bool:
        return self._state

class Timeseries:
    def __init__(self, max_len_s: float | None):
        self._latest_update_s = None
        self._max_len_s = max_len_s
        self._data = {}

    def add(self, t: float, value):
        self._data[t] = value
        if self._max_len_s:
            for ts in self._data:
                if t - ts > self._max_len_s:
                    self._data.pop(ts)
                else:
                    # exit the first time we encounter a timestamp within the length window
                    return
    
    def get(self, t: float) -> Tuple[float, Any]:
        # yeah, it's big brain time
        for a, b in grouped(self._data.keys(), 2):
            if abs(t - a) < abs(t - b):
                return a, self._data[a]
            else:
                return b, self._data[b]

    def get_latest(self):
        return self._data.values()[-1]