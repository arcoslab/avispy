#!/usr/bin/python
# -*- coding: utf-8 -*-
# Copyright (c) 2016 Universidad de Costa Rica,
# Autonomous Robots and Cognitive Systems Laboratory
# Authors: Israel Chaves Arbaiza <isracharbaiza@gmail.com>
#          Daniel García Vaglio <degv364@gmail.com>,
#          Federico Ruiz-Ugalde <memeruiz@gmail.com>
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
import time
from numpy import array, identity, sin, pi
from numpy.linalg import inv
from avispy.engine import (Camera, Scene, Light, Display, Primitive,
                           Object_model)
import avispy.objects_lib as objects_lib
import robot_desc_example as robot_desc


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

cylinder_obj = objects_lib.Cylinder()
cone_obj = objects_lib.Cone()

arrow = objects_lib.Arrow()
arrow.set_length(1.0)
arrow.set_color(array([1.0, 0., 0.]))
arrow.set_axis(array([0.0, 1., 2.]))

scene.add_object(arrow)

sphere_origin = objects_lib.Sphere()
sphere_origin.set_radius(0.1)
sphere_origin.set_pos(array([0., 0., 0.]))
sphere_origin.set_color(array([1.0, 1., 1.]))
scene.add_object(sphere_origin)

sphere_x = objects_lib.Sphere()
sphere_x.set_radius(0.1)
sphere_x.set_pos(array([1., 0., 0.]))
sphere_x.set_color(array([1.0, 0., 0.]))
scene.add_object(sphere_x)

sphere_y = objects_lib.Sphere()
sphere_y.set_radius(0.1)
sphere_y.set_pos(array([0., 1., 0.]))
sphere_y.set_color(array([0.0, 1., 0.]))
scene.add_object(sphere_y)

sphere_z = objects_lib.Sphere()
sphere_z.set_radius(0.1)
sphere_z.set_pos(array([0., 0., 1.]))
sphere_z.set_color(array([0.0, 0., 1.]))
scene.add_object(sphere_z)

sphere = objects_lib.Sphere()
sphere.set_radius(0.1)
sphere.set_pos(array([2., 2., 2.]))
sphere.set_color(array([1.0, 1., 0.]))
scene.add_object(sphere)

bar = objects_lib.Bar()
bar.set_length(3)
bar.set_sides(0.1, 0.1)
bar.set_pos(array([3., 0., 0.]))

curve = objects_lib.Curve()
curve2 = objects_lib.Curve()
curve2.set_y_offset(-2.0)
curve.set_color(array([1., 0., 0.]))
curve2.set_color(array([0., 1., 0.]))
curve.set_color_reflex(array([1., 0., 0.]), shininess=0.0)
curve2.set_color_reflex(array([0., 1., 0.]), shininess=0.0)

plot = objects_lib.Plot()
plot.add_curve(curve)
plot.set_pos(array([0.2, 0., 0.]))
plot.scale = [4., 4., 1.0]


articulated_right = objects_lib.Articulated(scene)
articulated_right2 = objects_lib.Articulated(scene)
articulated_left = objects_lib.Articulated(scene)
articulated_left2 = objects_lib.Articulated(scene)

for segment in robot_desc.arm_segments_right:
    gl_segment = objects_lib.Segment(segment)
    articulated_right.add_segment(gl_segment)
    gl_segment = objects_lib.Segment(segment)
    articulated_right2.add_segment(gl_segment)

for segment in robot_desc.arm_segments_left:
    gl_segment = objects_lib.Segment(segment)
    articulated_left.add_segment(gl_segment)
    gl_segment = objects_lib.Segment(segment)
    articulated_left2.add_segment(gl_segment)

tree = objects_lib.Articulated_tree()
tree.add_articulated(articulated_left, 0)
right_id = tree.add_articulated(articulated_right, 0)
tree.add_articulated(articulated_right2, right_id)
tree.add_articulated(articulated_left2, right_id)
robot_base = identity(4)
robot_base[:3][:, 3] = array([3., 0., 0.])
tree.trans_rot_matrix = robot_base
tree.update_transformations()

camera_center = objects_lib.Disk()
camera_center.set_color(array([0.5, 0.5, 0.5]))
camera_center.set_color_reflex(array([1., 1., 1.]), 50.0)
camera_center.visibility = False

scene.add_object(camera_center)

counter = 0.0
counter2 = -1.0

while True:
    time.sleep(0.001)
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and
                                         event.key == pygame.K_q):
            sys.exit()
        if event.type == pygame.MOUSEMOTION or \
           event.type == pygame.MOUSEBUTTONDOWN or \
           event.type == pygame.MOUSEBUTTONUP:
            camera.update(event)
            camera.camera_matrix.get_frame(
            )  # TODO, this is done twice innecessarly
            val = camera.camera_matrix.radius * 0.01
            camera_center.set_pos(
                inv(camera.camera_matrix.center_rot_frame)[:, 3][:3])
            camera_center.scale = [val, val, 1.0]
            if event.type == pygame.MOUSEMOTION:
                if event.buttons == (1, 0, 0) or event.buttons == (0, 0, 1):
                    camera_center.visibility = True
            if event.type == pygame.MOUSEBUTTONDOWN or \
               event.type == pygame.MOUSEBUTTONUP:
                if event.button == 4 or event.button == 5:
                    camera_center.visibility = True

    # testing curve
    counter += 0.1
    counter2 += 0.1
    if counter2 > 1.0:
        counter2 = -1.0
    curve.add_point(array([counter, counter2, 0.0]))
    curve2.add_point(array([counter, sin(counter), 0.0]))
    articulated_right.set_angles([counter] * 9)
    articulated_left.set_angles([counter - pi] * 9)
    articulated_left2.set_angles([counter - pi] * 9)
    tree.update_transformations()
    bar.set_length(sin(counter))
    display.update()
    camera_center.visibility = False
