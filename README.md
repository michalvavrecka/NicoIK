# NicoIK
Inverse kinematics for Nico robot. Win/Linux friendly.

## Instalation

You need just numpy and pybullet

`pip install pybullet`


If you want to work with real robot, you need o install NicoMotion

https://github.com/knowledgetechnologyuhh/NICO-software/tree/master/api/src/nicomotion

`python setup.py`

If you do not have real robot, you can comment lines 6 and 7 in ik_solver.py


## How to use

**Basic test**

`python ik_solver.py`

It has to output 7 joint angles in radians in repetitive manner

**Visualization**

`python ik_solver.py -g`

There is robot randomly calculating IK for finger on the line

**Left/right hand switch**

`python ik_solver.py -g -l`

The IK solver switch to left hand

**Custom position**

`python ik_solver.py -g -p 0.4 -0.3 0.3`

It will repeatedly calculate IK for this position and output joint angles to terminal

**Reset to initial position**

`python ik_solver.py -g -i`

It will reset robot to initial position defined in line 166 after each calculation

**Animate motion**

`python ik_solver.py -g -a`

It will show the trajectory of motion in simulator

**Real robot**

`python ik_solver.py -g -rr`

It will initialize robot to default position and then execute the motion from IK

**Custom movements**

If you want to put custom trajectory or other methods (from x,y,z to joint angles) write it to the target method instead of random generator

**Calibration matrix**

`python ik_solver.py -g -c -i -a`

[![Video1](https://img.youtube.com/vi/YWLnepOwRhc/maxresdefault.jpg)](https://youtu.be/YWLnepOwRhc)
