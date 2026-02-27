"""
by-wheel speed display utility for car movement

Revision History:
Version: date: changes:
         Feb 9  converted to function       
"""
__version__ = '0.2'
__date__ = 'Feb 9, 2023'
__author__ = 'Martin Baynes'

import math
import matplotlib.pyplot as plt
import numpy as np
import sys
from matplotlib import interactive
    
        
def plotWheelSpeed(wheel, speed, radius=1):
    mpspeed = valmap(speed, -100, 100, -90, 90)
    dx = radius * math.cos(math.radians(mpspeed))
    dy = radius * math.sin(math.radians(mpspeed))
    axd[wheel].quiver(0, 0, dy, dx, width=0.04,
                  pivot = 'mid', angles = 'uv', scale_units='height', scale=2)


   #equivalent of Arduino map()
def valmap(value, istart, istop, ostart, ostop): 
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))         


plt.style.use('_mpl-gallery-nogrid')
fig, axd = plt.subplot_mosaic([['left', 'left', 'upper middle','upper right' ],
                               ['left', 'left', 'lower middle', 'lower right']],
                               figsize=(8, 4), layout="constrained")
fig.suptitle('plt.subplot_mosaic()')

# plot

for idx, k in enumerate(axd):
    if idx > 0:
        axd[k].set(xlim=(-1.5, 1.5),  xticks=[], yticks=[],
        ylim=(-1, 1.5))
        axd[k].subtitle = idx
        
for idx, k in enumerate(axd):
    if idx > 0:
        if idx <= 2:
            plotWheelSpeed( k, idx * 50)
            print('Speed: ', idx * 50)
        elif idx == 3:
            plotWheelSpeed( k, -50)
            print('Speed: ', -50)
        elif idx == 4:
            plotWheelSpeed( k, -100)
            print('Speed: ', -100)

plt.show()
