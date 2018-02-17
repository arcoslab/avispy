#!/usr/bin/python
# -*- coding: utf-8 -*-
# Copyright (c) 2016 Universidad de Costa Rica,
# Autonomous Robots and Cognitive Systems Laboratory
# Authors: Federico Ruiz-Ugalde <memeruiz@gmail.com>
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

import sys
import pygame
from numpy import array, pi
from avispy.engine import Camera, Scene, Light, Display, \
    Primitive, Object_model
import time

xyz_inc = 0.1
rot_inc = 1 * pi / 180.0

camera = Camera()
size = 640, 480
scene = Scene()
display = Display(camera, scene, res=size)

light0 = Light(gl.GL_LIGHT0)
light0.position = array([10., 10., 10., 1.0])
scene.add_light(light0)

triangle_prim = Primitive()
triangle_prim.type = gl.GL_TRIANGLES
triangle_prim.vertices = array([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.]])
triangle_prim.normals = array([[0., 0., 1.], [0., 0., 1.], [0., 0., 1.]])
triangle_obj = Object_model()
triangle_obj.add_primitive(triangle_prim)
triangle_obj.material_diffuse_color = array([0., 1.0, 0., 1.])
triangle_obj.material_ambient_color = array([0., 0.2, 0., 1.])
triangle_obj.material_shininess = 50.
triangle_obj.material_specular_color = array([1., 1., 1., 1.])
triangle_obj.trans_rot_matrix[:, 3][:3] = array([4., 4., 0.])
triangle_obj.visibility = True
scene.add_object(triangle_obj)

while True:
    # for i in xrange(10):
    time.sleep(0.01)
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN
                                         and event.key == pygame.K_q):
            sys.exit()
        # camera events:
        if event.type == pygame.MOUSEMOTION or event.type == \
           pygame.MOUSEBUTTONDOWN or event.type == pygame.MOUSEBUTTONUP:
            camera.update(event)
    display.update()
