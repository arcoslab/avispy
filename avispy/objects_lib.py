#!/usr/bin/python
# -*- coding: utf-8 -*-
# Copyright (c) 2016 Universidad de Costa Rica, Autonomous Robots and Cognitive Systems Laboratory
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

import engine as  _engine
import numpy as _n
import numpy.linalg as _ln
from  numpy.random import random as _random
import OpenGL.GL as _gl 
import OpenGL.GLU as _glu
from scipy.spatial import ConvexHull


def gen_circle(radius, angle_step, z_pos):
    vertices=[]
    init_vertex=_n.array([radius,0.,z_pos])
    for angle in _n.arange(0.,360*_n.pi/180.0, angle_step):
        vertices.append(_n.array([radius*_n.sin(angle), radius*_n.cos(angle), z_pos]))
    return(vertices)
            
def gen_cylinder(radius,height,angle_step=10.0*_n.pi/180.0):
    vertices_base=gen_circle(radius, angle_step, 0)
    vertices_top=gen_circle(radius, angle_step, height)
    return(vertices_base,vertices_top)

def gen_convex_hull_faces(list_of_points):
    #gets a list of 3d points and returns a list of faces of the convex hull
    list_of_arrays=map(_n.array, list_of_points)
    hull=ConvexHull(list_of_arrays)
    return list(hull.points[hull.simplices])
    

class Disk(_engine.Object_model):
    def __init__(self, radius=1.0, angle_step=10.0*_n.pi/180.0):
        self.radius=radius
        primitive=_engine.Primitive()
        vertices=gen_circle(radius,angle_step,0.0)
        primitive.type=_gl.GL_TRIANGLE_FAN
        primitive.vertices=_n.array([_n.array([0.,0.,0.])]+vertices+[vertices[0]])
        primitive.normals=_n.array([0.,0.,1.])
        _engine.Object_model.__init__(self)
        self.add_primitive(primitive)
        

class Cone(_engine.Object_model):
    def __init__(self, radius=1.0, height=3.0, angle_step=10.0*_n.pi/180.0):
        self.radius=radius
        self.height=height
        #cone side surface
        self.side_surface_prim=_engine.Primitive()
        self.side_surface_prim.type=_gl.GL_TRIANGLE_STRIP
        vertices=gen_circle(radius,angle_step,0.0)
        tip=_n.array([0.,0.,height])
        vertices_temp=[]
        for vertex in vertices:
            vertices_temp.append(vertex)
            vertices_temp.append(tip)
        self.side_surface_prim.vertices=_n.array(vertices_temp+[vertices[0]])
        normals_temp=[]
        #for each vertex in the base circle calculate the normal
        for vertex in vertices:
            normal_temp=_n.cross(_n.cross(tip-vertex,vertex),tip-vertex)
            normal_temp/=_ln.norm(normal_temp)
            normals_temp.append(normal_temp)
        #calculate the normal for each "tip" as the average of  two contiguous vertices
        normals_tip=[]
        for i,normal in enumerate(normals_temp):
            next=i+1
            if i==len(normals_temp)-1:
                next=0
            normal_ave=_n.average(_n.array([normal,normals_temp[next]]),0)
            normals_tip.append(normal_ave/_ln.norm(normal_ave))
        normals=[]
        #putting all the normals together
        for normal_temp,normal_tip in zip(normals_temp, normals_tip):
            normals.append(normal_temp)
            normals.append(normal_tip)
        normals.append(normals[0])
        self.side_surface_prim.normals=_n.array(normals)
        #print "vertices", self.side_surface_prim.vertices
        #print "normals", self.side_surface_prim.normals
        
        #base cap surface
        self.base_cap_prim=_engine.Primitive()
        self.base_cap_prim.type=_gl.GL_TRIANGLE_FAN
        self.base_cap_prim.vertices=_n.array([_n.array([0.,0.,0.])]+vertices+[vertices[0]])
        self.base_cap_prim.normals=_n.array([0.,0.,-1.])
        
        _engine.Object_model.__init__(self)
        self.add_primitive(self.side_surface_prim)
        self.add_primitive(self.base_cap_prim)

    def set_scale(self, scale):
        self.scale=scale

class Cylinder(_engine.Object_model):
    def __init__(self, radius=1.0, height=1.0, angle_step=10.0*_n.pi/180.0):
        self.height=height
        self.radius=radius
        self.circular_surface_prim=_engine.Primitive()
        self.circular_surface_prim.type=_gl.GL_TRIANGLE_STRIP
        vertices_temp=[]
        normals_temp=[]
        vertices_base,vertices_top=gen_cylinder(radius, height, angle_step=angle_step)
        for vertex_base, vertex_top in zip(vertices_base,vertices_top):
            vertices_temp.append(vertex_base)
            vertices_temp.append(vertex_top)
            normals_temp.append(vertex_base/_ln.norm(vertex_base))
            normals_temp.append(vertex_base/_ln.norm(vertex_base))
        vertices_temp.append(vertices_base[0])
        vertices_temp.append(vertices_top[0])
        normals_temp.append(vertices_base[0]/_ln.norm(vertices_base[0]))
        normals_temp.append(vertices_base[0]/_ln.norm(vertices_base[0]))

        self.circular_surface_prim.vertices=_n.array(vertices_temp)
        self.circular_surface_prim.normals=_n.array(normals_temp)
        self.base_cap_prim=_engine.Primitive()
        self.base_cap_prim.type=_gl.GL_TRIANGLE_FAN
        self.base_cap_prim.vertices=_n.array([_n.array([0.,0.,0.])]+vertices_base+[vertices_base[0]])
        self.base_cap_prim.normals=_n.array([0.,0.,-1.])
        self.top_cap_prim=_engine.Primitive()
        self.top_cap_prim.type=_gl.GL_TRIANGLE_FAN
        self.top_cap_prim.vertices=_n.array([_n.array([0.,0.,height])]+vertices_top+[vertices_top[0]])
        self.top_cap_prim.normals=_n.array([0.,0.,1.])
        _engine.Object_model.__init__(self)
        self.add_primitive(self.circular_surface_prim)
        self.add_primitive(self.base_cap_prim)
        self.add_primitive(self.top_cap_prim)
        self.scale=[1.]*3

    def set_scale(self, scale):
        self.scale=scale


        

def calc_perpendicular_to_normal2(random_vector,normal):
    '''Calculates a perpendicular point to a normal'''
    #all this vectors are with respect to the origin (they are not rays)
    perp=_n.cross(normal,random_vector)
    perp/=_ln.norm(perp)
    return(perp)

def calc_different_random_vector(vector):
    random_vector=vector
    while _ln.norm(_n.cross(random_vector,vector))<0.000000001:
        random_vector=_random((3))
        random_vector*=2
        random_vector+=_n.array([-1,-1,-1])
        random_vector/=_ln.norm(random_vector)
    return(random_vector)


class Arrow(_engine.Object_model):
    def __init__(self):
        self.total_length=1.0
        self.len_shaft_cone_ratio=2.0
        self.shaft_length_width_ratio=12.0
        self.cone_length_width_ratio=3.0
        self.shaft_length=self.total_length/(1.0+(1.0/self.len_shaft_cone_ratio))
        self.cone_length=self.total_length-self.shaft_length
        _engine.Object_model.__init__(self)
        cylinder=Cylinder(radius=self.shaft_length/self.shaft_length_width_ratio,height=self.shaft_length)
        cone=Cone(radius=self.cone_length/self.cone_length_width_ratio, height=self.cone_length)
        cone.trans_rot_matrix[:,3][:3]=_n.array([0.,0.,cylinder.height])
        self.add_object_model(cylinder)
        self.add_object_model(cone)
        self.scale=[1.0,1.0,1.0]

    def set_length(self,length):
        self.scale[2]=length

    def set_relative_width(self,rel_width):
        self.scale[1]=rel_width
        self.scale[0]=self.scale[1]



class Sphere(_engine.Object_model):
    def __init__(self,radius=1.0, slices=20, stacks=20):
        _engine.Object_model.__init__(self)
        prim=_engine.Primitive()
        sphere_gl_list=_gl.glGenLists(1)
        quadric=_glu.gluNewQuadric()
        _gl.glNewList(sphere_gl_list,_gl.GL_COMPILE)
        _glu.gluSphere(quadric,radius,slices,stacks)
        _gl.glEndList()
        prim.gl_list=sphere_gl_list
        self.add_primitive(prim)
        self.scale=[1.,1.,1.]

    def set_radius(self,radius):
        self.scale[0]=self.scale[1]=self.scale[2]=radius

    def set_scale(self, scale):
        self.scale=scale

class Bar(_engine.Object_model):
    def __init__(self,side1=1.0,side2=1.0, height=1.0):
        _engine.Object_model.__init__(self)
        base_square_prim=_engine.Primitive()
        base_square_prim.type=_gl.GL_TRIANGLE_STRIP
        base_square_prim.vertices=_n.array([[-side1/2.0,-side2/2.0,0.0],
                                         [-side1/2.0,side2/2.0,0.0],
                                         [side1/2.0,-side2/2.0,0.0],
                                         [side1/2.0,side2/2.0,0.0]])
        base_square_prim.normals=_n.array([0.0,0.0,-1.0])
        self.add_primitive(base_square_prim)
        top_square_prim=_engine.Primitive()
        top_square_prim.type=_gl.GL_TRIANGLE_STRIP
        top_square_prim.vertices=_n.array([[-side1/2.0,-side2/2.0,height],
                                         [-side1/2.0,side2/2.0,height],
                                         [side1/2.0,-side2/2.0,height],
                                         [side1/2.0,side2/2.0,height]])
        top_square_prim.normals=_n.array([0.0,0.0,1.0])
        self.add_primitive(top_square_prim)
        side1a_square_prim=_engine.Primitive()
        side1a_square_prim.type=_gl.GL_TRIANGLE_STRIP
        side1a_square_prim.vertices=_n.array([[-side1/2.0,-side2/2.0,0.0],
                                         [-side1/2.0,-side2/2.0,height],
                                         [side1/2.0,-side2/2.0,0.0],
                                         [side1/2.0,-side2/2.0,height]])
        side1a_square_prim.normals=_n.array([0.0,1.0,0.0])
        self.add_primitive(side1a_square_prim)
        side1b_square_prim=_engine.Primitive()
        side1b_square_prim.type=_gl.GL_TRIANGLE_STRIP
        side1b_square_prim.vertices=_n.array([[-side1/2.0,side2/2.0,0.0],
                                         [-side1/2.0,side2/2.0,height],
                                         [side1/2.0,side2/2.0,0.0],
                                         [side1/2.0,side2/2.0,height]])
        side1b_square_prim.normals=_n.array([0.0,-1.0,0.0])
        self.add_primitive(side1b_square_prim)
        side2a_square_prim=_engine.Primitive()
        side2a_square_prim.type=_gl.GL_TRIANGLE_STRIP
        side2a_square_prim.vertices=_n.array([[-side1/2.0,-side2/2.0,0.0],
                                         [-side1/2.0,-side2/2.0,height],
                                         [-side1/2.0,side2/2.0,0.0],
                                         [-side1/2.0,side2/2.0,height]])
        side2a_square_prim.normals=_n.array([-1.0,0.0,0.0])
        self.add_primitive(side2a_square_prim)
        side2b_square_prim=_engine.Primitive()
        side2b_square_prim.type=_gl.GL_TRIANGLE_STRIP
        side2b_square_prim.vertices=_n.array([[side1/2.0,-side2/2.0,0.0],
                                         [side1/2.0,-side2/2.0,height],
                                         [side1/2.0,side2/2.0,0.0],
                                         [side1/2.0,side2/2.0,height]])
        side2b_square_prim.normals=_n.array([1.0,0.0,0.0])
        self.add_primitive(side2b_square_prim)
        self.scale=[1.0]*3

    def set_length(self,length):
        self.scale[2]=length

    def set_sides(self,side1,side2):
        self.scale[0]=side1
        self.scale[1]=side2

    def set_scale(self, scale):
        self.scale=scale

class Line(_engine.Object_model):
    def __init__(self,point1,point2):
        self.point1=point1
        self.point2=point2
        segment_prim=_engine.Primitive()
        segment_prim.type=_gl.GL_LINES
        segment_prim.vertices=_n.array([point1,point2])
        _engine.Object_model.__init__(self)
        self.add_primitive(segment_prim)


class Curve(_engine.Object_model):
    def __init__(self):
        _engine.Object_model.__init__(self)
        self.previous_point=None
        self.segments=[]
        self.center_offset_matrix=_n.identity(4)
        self.y_max=0.0
        self.y_min=0.0
        self.center_offset_matrix=_n.identity(4)
        self.zero_mark_id=self.add_object_model(Line(_n.array([-1.0,0.0,0.0]),_n.array([1.,0.,0.])))
        self.set_x_interval(1.0)
        self.set_y_offset(0.25)
        #self.material_shininess=0.0
        #self.material_specular_color=_n.array([1.,0.,0.,1.])

    def set_y_offset(self,y_offset):
        self.y_offset=y_offset
        self.center_offset_matrix[:3][:,3][1]=y_offset
        self.gen_gl_list()

    def set_x_interval(self,interval):
        self.x_interval=interval
        self.object_models[self.zero_mark_id].scale=[self.x_interval/(9*4),1.,1.]
        
    def add_point(self,point):
        if point[1]>self.y_max:
            self.y_max=point[1]
        if point[1]<self.y_min:
            self.y_min=point[1]
        if self.previous_point==None:
            self.previous_point=point
        else:
            segment=Line(self.previous_point,point)
            if len(self.segments)>0:
                first_point=self.object_models[self.segments[0]].point1
                while point[0]-first_point[0]>self.x_interval:
                    #print "Point", point, "First point", first_point
                    #print "Line ", self.segments[0], " removed"
                    self.remove_object_model(self.segments.pop(0))
                    first_point=self.object_models[self.segments[0]].point1
                    #print "Line ids", self.segments, first_point
                self.center_offset_matrix[:3][:,3][0]=-self.object_models[self.segments[0]].point1[0]
                self.object_models[self.zero_mark_id].trans_rot_matrix[:3][:,3][0]=self.object_models[self.segments[0]].point1[0]
            self.previous_point=point
            self.segments.append(self.add_object_model(segment))
            #print "Line point1", segment.point1
            #print "Line ", self.segments[-1], " added"
            self.gen_gl_list()


class Grid(_engine.Object_model):
    def __init__(self):
        _engine.Object_model.__init__(self)
        self.height=0.9
        self.length=1.0
        self.height_div=9
        self.length_div=10
        horizontal_segments=[Line(_n.array([0.,i,0.]),_n.array([self.length-self.length/self.length_div,i,0.])) for i in _n.arange(0.0,self.height,self.height/self.height_div)]
        map(self.add_object_model,horizontal_segments)
        vertical_segments=[Line(_n.array([i,0.,0.]),_n.array([i,self.height-self.height/self.height_div,0.])) for i in _n.arange(0.0,self.length,self.length/self.length_div)]
        map(self.add_object_model,vertical_segments)
        self.set_color(_n.array([0.5,0.5,0.5]))
        

class Plot(_engine.Object_model):
    def __init__(self):
        _engine.Object_model.__init__(self)
        self.curves=[]
        self.grid=Grid()
        self.y_div=0.5
        self.x_div=0.5
        #self.x_offset=0.0
        self.add_object_model(self.grid)

    def set_y_div(self,y_div):
        self.y_div=y_div
        for curve in self.curves:
            curve.scale=[self.grid.length/(self.x_div*self.grid.length_div),self.grid.height/(self.y_div*self.grid.height_div),1.]

    def set_x_div(self,x_div):
        self.x_div=x_div
        for curve in self.curves:
            curve.scale=[self.grid.length/(self.x_div*self.grid.length_div),self.grid.height/(self.y_div*self.grid.height_div),1.]

    def add_curve(self,curve):
        curve.scale=[self.grid.length/(self.x_div*self.grid.length_div),self.grid.height/(self.y_div*self.grid.height_div),1.]
        curve.trans_rot_matrix[:3][:,3][1]=((self.grid.height-self.grid.height/self.grid.height_div)/2) #curve.y_offset*(self.grid.height/(self.y_div*self.grid.height_div))
        curve.set_x_interval(self.x_div*(self.grid.length_div-1))
        self.curves.append(self.add_object_model(curve))

import PyKDL as kdl    

class Joint(Cylinder):
    RotX=1
    RotY=2
    RotZ=3
    none=8
    def __init__(self,joint_type):
        _engine.Object_model.__init__(self)
        self.type=joint_type
        if self.type==Joint.none:
            self.visibility=False

#class Rotation(object):
    

class Frame(_engine.Object_model):
    def __init__(self):
        _engine.Object_model.__init__(self)
        self.x_arrow=Arrow()
        self.y_arrow=Arrow()
        self.z_arrow=Arrow()
        self.x_arrow.set_color(_engine.Colors.red)
        self.x_arrow.set_axis(_n.array([1.,0,0]))
        self.y_arrow.set_color(_engine.Colors.green)
        self.y_arrow.set_axis(_n.array([0.,1.0,0]))
        self.z_arrow.set_color(_engine.Colors.blue)
        
        self.add_object_model(self.x_arrow)
        self.add_object_model(self.y_arrow)
        self.add_object_model(self.z_arrow)

    def set_scale(self, scale):
        #scale is a 3x1 array
        self.x_arrow.set_length(scale[0])
        self.y_arrow.set_length(scale[1])
        self.z_arrow.set_length(scale[2])

class Segment(_engine.Object_model):
    def __init__(self,kdl_segment):
        #'''kdl segment, first joint and then frame translation'''
        _engine.Object_model.__init__(self)

        self.joint_type=kdl_segment.getJoint().getTypeName()
        #link
        self.bar=Bar()
        self.kdl_link_frame=kdl_segment.getFrameToTip()
        self.link_trans=_n.array([self.kdl_link_frame.p[i] for i in xrange(3)])
        self.link_trans_rot=_n.identity(4)
        self.link_trans_rot[:3,:3]=_n.array([[self.kdl_link_frame.M[i,j] for j in xrange(3)] for i in xrange(3)])
        self.link_trans_rot[:,3][:3]=self.link_trans
        print "kdl Frame", self.link_trans_rot
        if self.joint_type=="TransY":
            #self.link_trans[1]=-0.2
            self.bar.set_axis(self.link_trans)
        else:
            self.bar.set_axis(self.link_trans)
        self.bar_id=self.add_object_model(self.bar)

        #joint
        self.joint_scale=kdl_segment.getJoint().getScale()
        print "Joint type", self.joint_type, self.joint_scale
        if self.joint_type!="None":
            print "create cylinder"
            self.cylinder=Cylinder()
            if self.joint_type=="RotX" or self.joint_type=="TransX":
                self.cylinder.set_axis(_n.array([1.,0.,0.]))
            if self.joint_type=="RotY" or self.joint_type=="TransY":
                self.cylinder.set_axis(_n.array([0.,1.,0.]))
            if self.joint_type=="RotZ" or self.joint_type=="TransZ":
                self.cylinder.set_axis(_n.array([0.,0.,1.]))
            self.cylinder.center_offset_matrix=_n.identity(4)
            self.cylinder.center_offset_matrix[2,3]=-0.5
            if self.joint_type=="TransY":
                self.cylinder.center_offset_matrix[2,3]=0.0
            self.cylinder_id=self.add_object_model(self.cylinder)

        #color
        if self.joint_type!="None":
            self.cylinder.set_color(_n.array([0.,1.,0]))
            self.cylinder.set_color_reflex(_n.array([1.,1.,1.]),50.0)
        if self.joint_type=="TransY":
            self.cylinder.set_color(_n.array([0.,0.,1]))
            self.cylinder.set_color_reflex(_n.array([1.,1.,1.]),50.0)
        self.bar.set_color(_n.array([1.,0.,0]))
        self.bar.set_color_reflex(_n.array([1.,1.,1.]),50.0)
        self.scale=_n.array([1.,1.,1.])

        #width
        bar_rel_width=0.1
        bar_width=_ln.norm(self.link_trans)*bar_rel_width
        self.set_width(bar_width)

    def set_width(self,width):
        self.bar.scale[0]=self.bar.scale[1]=width
        if self.joint_type!="None":
            self.cylinder.scale=[width*1.0,width*1.0,width*1.3]

    def set_angle(self,angle):
        self.joint_angle=angle
        if self.joint_type=="TransY":
            print "TransY set angle!"
            self.link_trans[1]=angle
            self.bar.set_axis(self.link_trans)
            self.bar.gen_gl_list()
            self.cylinder.scale=[self.bar.scale[0], self.bar.scale[0], abs(angle)]

class Articulated():
    def __init__(self,scene):
        self.base_frame=_n.identity(4)
        self.segments=[]
        self.scene=scene
        self.angles=[]
        self.trans_rot_matrix=_n.identity(4)
        self.last_trans_rot=_n.identity(4)

    def set_base_frame(self, frame):
        self.trans_rot_matrix=frame

    def add_segments(self, segments,width=0.03):
        for segment in segments:
            self.angles.append(0.0)
            self.add_segment(Segment(segment),width=width)

    def add_segment(self, segment,width=0.03):
        segment.set_width(width)
        self.segments.append(segment)
        self.scene.add_object(segment)
        self.angles.append(0.0)
        self.update_transformations()

    def set_angles(self, angles):
        #print "Angles", angles
        self.angles=[]
        i=0
        #creating the complete angle list (including None joint angles)
        for segment in self.segments:
            #print segment.joint_type
            if segment.joint_type=="None":
                self.angles.append(0.0)
            else:
                if len(angles)==0:
                    print "Angle list to small"
                else:
                    self.angles.append(angles[i]*segment.joint_scale)
                    i+=1
        if len(self.angles)!=len(self.segments):
            print "Error, wrong number of angles, got", len(self.angles), " should be:", len(self.segments)
            self.angles=[0.0]*len(self.segments)

    def update_transformations(self):
        next_link_trans_rot=self.trans_rot_matrix
        for segment,angle in zip(self.segments,self.angles):
            rot_frame=_n.identity(4)
            if segment.joint_type=="RotX":
                rot_frame[:3,:3]=_engine.rotx(angle)
            if segment.joint_type=="RotY":
                rot_frame[:3,:3]=_engine.roty(angle)
            if segment.joint_type=="RotZ":
                rot_frame[:3,:3]=_engine.rotz(angle)
            if segment.joint_type=="TransY":
                segment.link_trans_rot[1,3]=angle
                segment.scale[1]=angle*26.0
                segment.gen_gl_list()
                segment.object_models[1].gen_gl_list()
                segment.bar.gen_gl_list()
            segment.trans_rot_matrix=_n.dot(next_link_trans_rot,rot_frame)
            next_link_trans_rot=_n.dot(segment.trans_rot_matrix,segment.link_trans_rot)
        self.last_trans_rot=next_link_trans_rot

class Articulated_tree(object):
    def __init__(self):
        self.articulates={}
        self.articulated_id=0
        self.tree={}
        self.trans_rot_matrix=_n.identity(4)

    def add_articulated(self,articulated,parent):
        self.articulated_id+=1
        self.articulates[self.articulated_id]=articulated
        child=self.articulated_id
        if not (parent in self.tree):
            self.tree[parent]=[child]
        else:
            self.tree[parent]+=[child]
        return(child)

    def update_transformations(self):
        for parent in self.tree.keys():
            childs=self.tree[parent]
            for child in childs:
                #set base frame of child to parent one
                if parent==0:
                    parent_frame=self.trans_rot_matrix
                else:
                    parent_frame=self.articulates[parent].last_trans_rot
                self.articulates[child].set_base_frame(parent_frame)
                self.articulates[child].update_transformations()

### Framed objects

class Framed_face(_engine.Object_model):
    def __init__(self, width=0.1, height=0.1, step=0.02, angular_step=_n.pi/6):
        _engine.Object_model.__init__(self)
        self.list_of_frames=[]
        self.step=step
        self.angular_step=angular_step
        self.x_start=-width/2.0
        self.x_end=width/2.0
        self.y_start=-height/2.0
        self.y_end=height/2.0
        self.width=width
        self.height=height
        self.scale=[1.0,1.0,1.0]
        self.generate_frames()

    def generate_frames(self):
        x_sample_cant=int(self.width/self.step)
        y_sample_cant=int(self.height/self.step)
        theta_sample_cant=int(2*_n.pi/self.angular_step)
        x=self.x_start
        y=self.y_start
        theta=0

        for i in xrange(x_sample_cant+1):
            y=self.y_start
            theta=0
            for j in xrange(y_sample_cant+1):
                theta=0
                for k in xrange(theta_sample_cant+1):
                    pose_matrix=_n.identity(4)
                    pose_matrix[:3,:3]=_engine.rotz(theta)
                    pose_matrix[:3,3]=_n.array([x,y,0])
                    new_frame=Frame()
                    new_frame.scale=[0.01,0.01,0.01]
                    new_frame.set_trans_rot_matrix(pose_matrix)
                    self.add_object_model(new_frame)
                    theta+=self.angular_step
                y+=self.step
            x+=self.step
        

        
class Framed_sphere(_engine.Object_model):
    def __init__(self, radius=0.1, step=_n.pi/6, start=0, end=2*_n.pi):
        _engine.Object_model.__init__(self)
        self.list_of_frames=[]
        self.step=step
        self.start=start
        self.end=end
        self.radius=radius
        self.position=self.trans_rot_matrix[:3,3]
        self.generate_frames()
        self.scale=[1.0,1.0,1.0]

    def generate_frames(self):
        #this will generate the angles
        #the following are the angles (like in polar coordinates)
        phi=0   #from 0 to pi/2
        theta=0 #from 0 to pi
        psi=0 # from 0 to 2pi but half resolution
        phi_sample_cant=int((self.end-self.start)/self.step)
        theta_sample_cant=int(_n.pi/self.step)
        psi_sample_cant=int(_n.pi/self.step)
        pose_matrix=_n.identity(4)
        rel_pos_ini=_n.array([0.0,0.0,1.0]) #relative pose
    
        self.list_of_poses=[]
        for i in xrange(phi_sample_cant+1):
            theta=0
            psi=0
            for j in xrange(theta_sample_cant+1):
                psi=0
                for k in xrange(psi_sample_cant):
                    #this will iterate over the angles, now generate the poses.
                    #generate orientation part
                    rot_matrix=_n.dot(_n.dot(_engine.rotx(phi-_n.pi/2), _engine.roty(theta-_n.pi/2)), _engine.rotz(psi))
                    rel_pos=_n.dot(rot_matrix, rel_pos_ini)
                    pos=self.position-self.radius*rel_pos
                    pose_matrix=_n.identity(4)
                    pose_matrix[:3,:3]=rot_matrix
                    pose_matrix[:3,3]=pos
                    new_frame=Frame()
                    new_frame.scale=[.01,.01,.01]
                    new_frame.set_trans_rot_matrix(pose_matrix)
                    self.add_object_model(new_frame)
                    #self.list_of_frames.append(new_frame) 
                    psi+=self.step;
                theta+=self.step;
            phi+=self.step;

class Framed_box(_engine.Object_model):
    def __init__(self, width=0.2, height=0.1, depth=0.3, step=0.04, angle_step=_n.pi/6):
        _engine.Object_model.__init__(self)
        self.step=step
        self.angle_step=angle_step
        self.x_length=width
        self.y_length=height
        self.z_length=depth

        #create pose matrices
        xy_top_pose=_n.identity(4)
        xy_bottom_pose=_n.identity(4)
        
        xz_right_pose=_n.identity(4)
        xz_left_pose=_n.identity(4)
        
        yz_front_pose=_n.identity(4)
        yz_back_pose=_n.identity(4)

        #orientation
        xy_bottom_pose[:3,:3]=_engine.rotx(_n.pi)
        
        xz_right_pose[:3,:3]=_engine.rotx(-_n.pi/2)
        xz_left_pose[:3,:3]=_engine.rotx(_n.pi/2)

        yz_front_pose[:3,:3]=_engine.roty(_n.pi/2)
        yz_back_pose[:3,:3]=_engine.roty(-_n.pi/2)

        #position
        xy_top_pose[:3,3]=_n.array([0,0,self.z_length/2])
        xy_bottom_pose[:3,3]=_n.array([0,0,-self.z_length/2])
        
        xz_right_pose[:3,3]=_n.array([0,self.y_length/2,0])
        xz_left_pose[:3,3]=_n.array([0,-self.y_length/2,0])
        
        yz_front_pose[:3,3]=_n.array([self.x_length/2,0,0])
        yz_back_pose[:3,3]=_n.array([-self.x_length/2,0,0])

        #visualization
        xy_top_face=Framed_face(self.x_length, self.y_length, self.step, self.angle_step)
        xy_bottom_face=Framed_face(self.x_length, self.y_length, self.step, self.angle_step)
        
        xz_right_face=Framed_face(self.x_length, self.z_length, self.step, self.angle_step)
        xz_left_face=Framed_face(self.x_length, self.z_length, self.step, self.angle_step)
        
        yz_front_face=Framed_face(self.z_length, self.y_length, self.step, self.angle_step)
        yz_back_face=Framed_face(self.z_length, self.y_length, self.step, self.angle_step)


        xy_top_face.set_trans_rot_matrix(xy_top_pose)
        xy_bottom_face.set_trans_rot_matrix(xy_bottom_pose)
        
        xz_right_face.set_trans_rot_matrix(xz_right_pose)
        xz_left_face.set_trans_rot_matrix(xz_left_pose)
        
        yz_front_face.set_trans_rot_matrix(yz_front_pose)
        yz_back_face.set_trans_rot_matrix(yz_back_pose)

        self.add_object_model(xy_top_face)
        self.add_object_model(xy_bottom_face)
        self.add_object_model(xz_right_face)
        self.add_object_model(xz_left_face)
        self.add_object_model(yz_front_face)
        self.add_object_model(yz_back_face)

        
class Convex_hull(_engine.Object_model):
    def __init__(self, list_of_points):
        _engine.Object_model.__init__(self)
        self.faces=gen_convex_hull_faces(list_of_points)
        for face in self.faces:
            normal=_n.cross(-face[0,:3]+face[1,:3], -face[0,:3]+face[2:3])
            prim=_engine.Primitive()
            prim.type=_gl.GL_TRIANGLES
            prim.vertices=face
            prim.normals=_n.identity(3)
            prim.normals[0,:3]=normal
            prim.normals[1,:3]=normal
            prim.normals[2,:3]=normal
            self.add_primitive(prim)

    def set_scale(self, scale):
        self.scale=scale


class Point_set_surface(_engine.Object_model):
    def __init__(self, list_of_points):
        _engine.Object_model.__init__(self)



class Directed_cylinder(_engine.Object_model):
    def __init__(self, radius=1.0, height=1.0, angle_step=10.0*_n.pi/180.):
        _engine.Object_model.__init__(self)
        self.cylinder=Cylinder(radius=radius,height=height)
        self.frame=Frame()
        
        frame_pose=_n.identity(4)
        frame_pose[2,3]=height*1.2
        self.frame.set_trans_rot_matrix(frame_pose)
        self.add_object_model(self.cylinder)
        self.add_object_model(self.frame)
    def set_scale(self, scale):
        self.cylinder.set_scale(scale)
        self.frame.set_scale(scale)

class Directed_bar(_engine.Object_model):
    def __init__(self, side1=1.0,side2=1.0, height=1.0):
        _engine.Object_model.__init__(self)
        self.box=Bar(side1=side1, side2=side2, height=height)
        self.frame=Frame()
        
        frame_pose=_n.identity(4)
        frame_pose[2,3]=height*1.2

        self.frame.set_trans_rot_matrix(frame_pose)
        self.add_object_model(self.box)
        self.add_object_model(self.frame)


    def set_scale(self, scale):
        self.box.set_scale(scale)
        self.frame.set_scale(scale)

        
        
