#!/usr/bin/python

import OpenGL.GL as gl

import sys, pygame
from numpy import array, identity, cos , sin, dot, invert, pi
from avispy.engine import Camera, Scene, Light, Display, Primitive, Object_model

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

while True:
#for i in xrange(10):
    time.sleep(0.01)
    for event in pygame.event.get():
        print event
        if event.type == pygame.QUIT or (event.type ==pygame.KEYDOWN and event.key == pygame.K_q):
            sys.exit()
        #camera events:
        if event.type == pygame.MOUSEMOTION or event.type ==pygame.MOUSEBUTTONDOWN or event.type == pygame.MOUSEBUTTONUP:
            camera.update(event)
    display.update()

    
