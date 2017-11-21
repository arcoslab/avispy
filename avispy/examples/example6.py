#!/usr/bin/python
# -*- coding: utf-8 -*-
# Copyright (c) 2016 Universidad de Costa Rica, Autonomous Robots and Cognitive Systems Laboratory
# Authors: Israel Chaves Arbaiza <isracharbaiza@gmail.com>, Daniel García Vaglio <degv364@gmail.com>, Federico Ruiz-Ugalde <memeruiz@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import OpenGL.GL as gl

import sys, pygame
from numpy import array, identity, cos , sin, dot, invert, pi
from avispy.engine import Camera, Scene, Light, Display, Primitive, Object_model, rotx, roty, rotz
import avispy.objects_lib as objects_lib
from numpy.random import rand

xyz_inc=0.1
rot_inc=1*pi/180.0

import time
camera=Camera()
size= 640,480
scene=Scene()
display=Display(camera,scene,res=size)

light0=Light(Light.LIGHTS[0])
light0.position=array([10.,10.,10.,1.0])
scene.add_light(light0)

light1=Light(Light.LIGHTS[1])
light1.position=array([-10.,10.,10.,1.0])
scene.add_light(light1)

light2=Light(Light.LIGHTS[2])
light2.position=array([0.,-10.,10.,1.0])
scene.add_light(light2)

light3=Light(Light.LIGHTS[3])
light3.position=array([0.,0.,-10.,1.0])
scene.add_light(light3)


camera_center=objects_lib.Disk()
camera_center.set_color(array([0.5,0.5,0.5]))
camera_center.set_color_reflex(array([1.,1.,1.]),50.0)
camera_center.visibility=False

scene.add_object(camera_center)

#Adding objects
world_frame=objects_lib.Frame()
#scene.add_object(world_frame)
test_frame=identity(4)

#Add convex hull

points=rand(50, 3)
convex_hull=objects_lib.Convex_hull(points)
scene.add_object(convex_hull)


counter=0.0
counter2=-1.0

from math import sin

from numpy.linalg import inv

while True:
#for i in xrange(10):
    time.sleep(0.01)
    for event in pygame.event.get():
        #print event
        if event.type == pygame.QUIT or (event.type ==pygame.KEYDOWN and event.key == pygame.K_q):
            sys.exit()
        #camera events:
        if event.type == pygame.MOUSEMOTION or event.type ==pygame.MOUSEBUTTONDOWN or event.type == pygame.MOUSEBUTTONUP:
            camera.update(event)
            camera.camera_matrix.get_frame() #TODO, this is done twice innecessarly
            val=camera.camera_matrix.radius*0.01
            camera_center.set_pos(inv(camera.camera_matrix.center_rot_frame)[:,3][:3])
            camera_center.scale=[val,val,1.0]
            if event.type == pygame.MOUSEMOTION :
                if event.buttons == (1,0,0) or event.buttons == (0,0,1):
                    camera_center.visibility=True
            if event.type == pygame.MOUSEBUTTONDOWN or event.type == pygame.MOUSEBUTTONUP:
                if event.button == 4 or event.button ==5:
                    camera_center.visibility=True

    #testing curve
    counter+=0.1
    counter2+=0.1
    if counter2>1.0:
        counter2=-1.0
    test_frame[:3][:,:3]=rotx(counter)
    world_frame.trans_rot_matrix=test_frame
    display.update()
    camera_center.visibility=False

    
