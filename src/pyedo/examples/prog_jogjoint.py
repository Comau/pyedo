#!/usr/bin/env python

import pyedo
import pyedo.edo

#EXAMPLE of Program with CARTESIAN MOVE
def prog_jogjoint(edo):
    myedo = edo
    for x in range(0, 300):
        myedo.jog_joint(100,0,0,0,0,0,-1,0)
        print('x == {0}',x)

if __name__ == 'prog_jogjoint':
    prog_jogjoint()