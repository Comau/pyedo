#!/usr/bin/env python

from pyedo import edo

#EXAMPLE of Program with JOINT MOVE 

def rodeo(edo,ovr = 70):
    myedo = edo
    myedo.move_joint(ovr, 90, 52.3, -90.8, 0, 38.5, -90, 0.61)
    myedo.move_joint(ovr, -45, -52.3, 90.8, 0, -38.5, 45, 0.61)
    myedo.move_joint(ovr, 60, 30, 95, 0, -45, 0, 80)
    myedo.move_joint(ovr, -30, 90, -40, -90, -90, 0, 0.61)
    myedo.move_joint(ovr, 60, 40, -40, 90, -90, 0, 0.61)
    myedo.move_joint(ovr, 0, -40, 80, 0, 80, 0, 80)
    myedo.move_joint(100)

if __name__ == 'rodeo':
    rodeo(edo)

