#!/usr/bin/env python

import pyedo
import pyedo.edo

#EXAMPLE of Program with CIRCULAR EXTENDED MOVE
def prog_circularX(edo,ovr=50):
    myedo = edo
    myedo.move_joint(ovr,0,45,45,0,90,0,0)
    myedo.move_cartesian(ovr, 417, 0, 180, 0, 180, 0)
    myedo.move_circularX(ovr ,447, 0, 180, 0, 180, 0, 70, 432, 15, 180, 0, 180, 0, 0)
    myedo.move_circularX(ovr ,417, 0, 180, 0, 180, 0, 0, 432,-15, 180, 0, 180, 0, 0)

if __name__ == 'prog_circularX':
    prog_circularX(edo)