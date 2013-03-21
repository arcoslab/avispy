from PyKDL import *
from math import pi
from numpy import array

# arm configuration
arm_segments_right = [
        Segment(Joint(Joint.None),
            Frame(Rotation.RotY(-120.0*pi/180.0)*Rotation.RotX(pi/2),Vector(0.05,-0.245,1.338))),
        Segment(Joint(Joint.None),
            Frame(Rotation.Identity(), Vector(0.0, 0.0, 0.11))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(-pi/2), Vector(0.0, 0.0, 0.20))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(pi/2), Vector(0.0, -0.20, 0.0))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(pi/2), Vector(0, 0, .20))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(-pi/2), Vector(0, 0.2, 0))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(-pi/2), Vector(0, 0, 0.19))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(pi/2), Vector(0, -0.078, 0.0))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotZ(3*pi/4)*Rotation.RotX(pi/2), Vector(0.075, 0.075, -0.094))),
#        Segment(Joint(Joint.None),
#            Frame(Rotation.Identity(), Vector(0.07, -0.025, 0.28))),
            ]

arm_segments_left = [
        Segment(Joint(Joint.None),
            Frame(Rotation.RotX(-pi/2),Vector(0.05,0.245,1.338))),
        Segment(Joint(Joint.None),
            Frame(Rotation.Identity(), Vector(0.0, 0.0, 0.11))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(-pi/2), Vector(0.0, 0.0, 0.20))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(pi/2), Vector(0.0, -0.20, 0.0))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(pi/2), Vector(0, 0, .20))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(-pi/2), Vector(0, 0.2, 0))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(-pi/2), Vector(0, 0, 0.19))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(pi/2), Vector(0, -0.078, 0.0))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotZ(pi*3/4)*Rotation.RotX(-pi/2)*Rotation.RotY(pi/2), Vector(-0.075, 0.075, -0.094))),
#        Segment(Joint(Joint.None),
#            Frame(Rotation.Identity(), Vector(0.07, -0.025, 0.28))),
            ]
