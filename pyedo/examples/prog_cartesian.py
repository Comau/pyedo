#!/usr/bin/env python

import pyedo
import pyedo.edo

#EXAMPLE of Program with CARTESIAN MOVE
def prog_cartesian(edo,ovr=50):
    myedo = edo
    myedo.move_joint(ovr,0,45,45,0,90,0,0)
    myedo.move_cartesian(ovr, 400, 0, 250, 0, 180, 0)
    myedo.move_cartesian(ovr, 400, -100, 180, 0, 180, 0)
    myedo.move_cartesian(ovr, 400, -100, 120, 0, 180, 0)
    myedo.move_cartesian(ovr, 400, -100, 180, 0, 180, 0)
    myedo.move_cartesian(ovr, 400, 200, 180, 0, 180, 0)
    myedo.move_cartesian(ovr, 400, 200, 120, 0, 180, 0)
    myedo.move_cartesian(ovr, 400, 200, 180, 0, 180, 0)
    myedo.move_cartesian(ovr, 400, -200, 180, 0, 180, 0)
    myedo.move_cartesian(ovr, 400, -100, 180, 0, 180, 0)
    myedo.move_cartesian(ovr, 400, -100, 120, 0, 180, 0)
    
if __name__ == 'prog_cartesian':
    prog_cartesian(edo)