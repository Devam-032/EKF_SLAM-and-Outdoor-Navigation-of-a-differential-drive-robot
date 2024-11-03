#!/usr/bin/env python3

import numpy as np

a = np.array([
    [1,2,3],
    [3,4,5]
])
(b,c)=a.shape
d = np.eye(b+1,c+1)
print(d)