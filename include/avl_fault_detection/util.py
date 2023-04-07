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

def kill_all_nodes():
    if len(os.popen("rosnode list").readlines()) > 0:
        os.system('rosnode kill -a')

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

class MovingAvg:

    def __init__(self, window_size: int):
        self._window_size = window_size
        self._window = []
    
    def avg(self) -> float:
        return sum(self._window) / len(self._window)
    
    def var(self) -> float:
        avg = self.avg()
        return sum((x - avg)**2 for x in self._window) / len(self._window)

    def std(self) -> float:
        avg = self.avg()
        return self.var()**0.5   

    def add(self, val: float) -> float:
        self._window.append(val)
        if len(self._window) > self._window_size:
            self._window.pop(0)
        return self.avg()
    
    def reset(self):
        self._window = []
        self._sum = 0

class MultiMovingAvg:

    def __init__(self, window_size: int, n: int):
        self._n = n
        self._moving_avg = [MovingAvg(window_size) for _ in range(n)]
    
    def avg(self) -> Tuple[float]:
        return tuple(m.avg() for m in self._moving_avg) 
    
    def var(self) -> Tuple[float]:
        return tuple(m.var() for m in self._moving_avg)

    def std(self) -> Tuple[float]:
        return tuple(m.std() for m in self._moving_avg)

    def add(self, vals: Tuple[float]) -> Tuple[float]:
        return tuple(m.add(v) for m, v in zip(self._moving_avg, vals))

    def reset(self):
        for m in self._moving_avg:
            m.reset()