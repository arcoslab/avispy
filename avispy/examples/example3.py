#!/usr/bin/python

import OpenGL.GL as gl

import sys, pygame
from numpy import array, identity, cos , sin, dot, invert, pi
from vispy.engine import Camera, Scene, Light, Display, Primitive, Object_model, rotx, roty, rotz
import vispy.objects_lib as objects_lib

xyz_inc=0.1
rot_inc=1*pi/180.0

import time
camera=Camera()
size= 640,480
scene=Scene()
display=Display(camera,scene,res=size)

light0=Light(gl.GL_LIGHT0)
light0.position=array([10.,10.,10.,1.0])
scene.add_light(light0)

triangle_prim=Primitive()
triangle_prim.type=gl.GL_TRIANGLES
triangle_prim.vertices=array([[0.,0.,0.],
                        [1.,0.,0.],
                        [1.,1.,0.]])
triangle_prim.normals=array([[0.,0.,1.],
                       [0.,0.,1.],
                       [0.,0.,1.]])
triangle_obj=Object_model()
triangle_obj.add_primitive(triangle_prim)
triangle_obj.material_diffuse_color=array([0.,1.0,0.,1.])
triangle_obj.material_ambient_color=array([0.,0.2,0.,1.])
triangle_obj.material_shininess=50.
triangle_obj.material_specular_color=array([1.,1.,1.,1.])
triangle_obj.trans_rot_matrix[:,3][:3]=array([4.,4.,0.])
triangle_obj.visibility=True
scene.add_object(triangle_obj)

cylinder_obj=objects_lib.Cylinder()
cone_obj=objects_lib.Cone()
arrow=objects_lib.Arrow()
#scene.add_object(cylinder_obj)
#scene.add_object(cone_obj)
arrow.set_length(1.0)
#arrow.set_pos(array([-2.,0.,0.]))
arrow.set_color(array([1.0,0.,0.]))
arrow.set_axis(array([0.0,1.,2.]))
#arrow.trans_rot_matrix[:3,:3]=rotx(90*pi/180.0)
scene.add_object(arrow)

sphere_origin=objects_lib.Sphere()
sphere_origin.set_radius(0.1)
sphere_origin.set_pos(array([0.,0.,0.]))
sphere_origin.set_color(array([1.0,1.,1.]))
scene.add_object(sphere_origin)

sphere_x=objects_lib.Sphere()
sphere_x.set_radius(0.1)
sphere_x.set_pos(array([1.,0.,0.]))
sphere_x.set_color(array([1.0,0.,0.]))
scene.add_object(sphere_x)

sphere_y=objects_lib.Sphere()
sphere_y.set_radius(0.1)
sphere_y.set_pos(array([0.,1.,0.]))
sphere_y.set_color(array([0.0,1.,0.]))
scene.add_object(sphere_y)

sphere_z=objects_lib.Sphere()
sphere_z.set_radius(0.1)
sphere_z.set_pos(array([0.,0.,1.]))
sphere_z.set_color(array([0.0,0.,1.]))
scene.add_object(sphere_z)

sphere=objects_lib.Sphere()
sphere.set_radius(0.1)
sphere.set_pos(array([2.,2.,2.]))
sphere.set_color(array([1.0,1.,0.]))
scene.add_object(sphere)

bar=objects_lib.Bar()
bar.set_length(3)
bar.set_sides(0.1,0.1)
bar.set_pos(array([3.,0.,0.]))
#scene.add_object(bar)

curve=objects_lib.Curve()
curve2=objects_lib.Curve()
curve2.set_y_offset(-2.0)
curve.set_color(array([1.,0.,0.]))
curve2.set_color(array([0.,1.,0.]))
curve.set_color_reflex(array([1.,0.,0.]),shininess=0.0)
curve2.set_color_reflex(array([0.,1.,0.]),shininess=0.0)
#curve.set_x_interval(2.0)
plot=objects_lib.Plot()
plot.add_curve(curve)
#plot.add_curve(curve2)
plot.set_pos(array([0.2,0.,0.]))
plot.scale=[4.,4.,1.0]
#scene.add_object(plot)


import PyKDL as kdl
articulated=objects_lib.Articulated(scene)

segment=kdl.Segment(kdl.Joint(kdl.Joint.None),kdl.Frame(kdl.Rotation.RotX(0*pi/180.0),kdl.Vector(1,0.,5)))
segment=objects_lib.Segment(segment)
#segment.set_pos(array([-3.,0.,0.]))
#scene.add_object(segment)
articulated.add_segment(segment)

segment=kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Rotation.RotX(0*pi/180.0),kdl.Vector(1,0,0)))
segment=objects_lib.Segment(segment)
#segment.set_pos(array([-3.,0.,0.]))
#scene.add_object(segment)
articulated.add_segment(segment)

segment=kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Rotation.RotX(0*pi/180.0),kdl.Vector(0,0,-1)))
segment=objects_lib.Segment(segment)
#segment.set_pos(array([-3.,0.,0.]))
#scene.add_object(segment)
articulated.add_segment(segment)

camera_center=objects_lib.Disk()
camera_center.set_color(array([0.5,0.5,0.5]))
camera_center.set_color_reflex(array([1.,1.,1.]),50.0)
camera_center.visibility=False

scene.add_object(camera_center)

#grid=objects_lib.Grid()
#scene.add_object(grid)

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
    curve.add_point(array([counter,counter2,0.0]))
    curve2.add_point(array([counter,sin(counter),0.0]))
    articulated.set_angles([counter,counter,counter*2])
    bar.set_length(sin(counter))
    display.update()
    camera_center.visibility=False

    
