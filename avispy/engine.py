#!/usr/bin/python
import OpenGL.GL as _gl
import OpenGL.GLU as _glu
import time, os

import pygame as _pygame
import numpy as _n
import numpy.linalg as _ln

def rotx(angle):
    '''Generates a rotational matrix rotated angle around x axis'''
    return(_n.array([[1,0,0],
                  [0,_n.cos(angle),-_n.sin(angle)],
                  [0,_n.sin(angle),_n.cos(angle)]]))
def roty(angle):
    '''Generates a rotational matrix rotated angle around y axis'''
    return(_n.array([[_n.cos(angle),0,_n.sin(angle)],
                  [0,1,0],
                  [-_n.sin(angle),0,_n.cos(angle)]]))
def rotz(angle):
    '''Generates a rotational matrix rotated angle around z axis'''
    return(_n.array([[_n.cos(angle),-_n.sin(angle),0],
                  [_n.sin(angle),_n.cos(angle),0],
                  [0,0,1]]))


class Colors():
    red=_n.array([1.0,0.,0.])
    green=_n.array([0.,1.,0.])
    blue=_n.array([0.,0.,1.])

class Primitive(object):
    type=None #GL_QUAD, GL_QUADS, GL_TRIANGLE, etc
    vertices=None #a numpy list of vertices
    normals=None
    trans_rot_matrix=None #if this primitive has to be translated and/or rotated put matrix here
    #if this primitive has a different color than the rest of the object then defined it here
    material_specular_color=None 
    material_shininess=None
    material_ambient_color=None
    material_diffuse_color=None
    gl_list=None #we can also include a gl_list (this gets executed at the end of the primitive

def calc_perpendicular_to_normal(normal):
    if (abs(normal[2]) > abs(normal[1])) and (abs(normal[2]) > abs(normal[0])):
        return(_n.array([0.0,normal[2],-normal[1]]))
    else:
        return(_n.array([normal[1],-normal[0],0.]))


class Object_model(object):
    def __init__(self):
        #set default values for a Null object
        self.visibility=True
        self.center_offset_matrix=None 
        self.trans_rot_matrix=_n.identity(4)
        self.scale=None
        self.material_specular_color=None
        self.material_shininess=None
        self.material_ambient_color=None
        self.material_diffuse_color=None
        self.gl_primitives={}
        self.object_models={}
        self.counter=-1
        self.object_model_counter=-1
        self.gl_list=_gl.glGenLists(1)
        self.color=None
        #self.gen_gl_list()

    def __del__(self):
        _gl.glDeleteLists(self.gl_list,1)
        #print "Destroying object"

    def add_object_model(self,object_model):
        '''Includes an object model inside the current object model'''
        self.object_model_counter+=1
        self.object_models[self.object_model_counter]=object_model
        return(self.object_model_counter)

    def remove_object_model(self,object_model_id):
        if object_model_id in self.object_models:
            del(self.object_models[object_model_id])
        else:
            print "Object models not present in object model list"

    def set_color(self,color):
        self.material_ambient_color=_n.concatenate((color,_n.array([1.])))
        self.material_diffuse_color=_n.concatenate((color,_n.array([1.])))

    def set_color_reflex(self,color,shininess=50.0):
        self.material_specular_color=_n.concatenate((color,_n.array([1.])))
        self.material_shininess=shininess

    def set_axis(self,axis):
        #create matrix out of axis using a random vector and projecting it into the perpendicular plane of the axis, the axis is the third column or row of the rot matrix
        self.axis=axis/_ln.norm(axis)
        length=_ln.norm(axis)
        perp_vector=calc_perpendicular_to_normal(self.axis)
        last_vector=_n.cross(self.axis,perp_vector)
        self.trans_rot_matrix[:3,0]=last_vector
        self.trans_rot_matrix[:3,1]=perp_vector
        self.trans_rot_matrix[:3,2]=self.axis
        self.scale[2]=length

    def set_pos(self,pos):
        self.trans_rot_matrix[:,3][:3]=pos

    def add_primitive(self,primitive):
        '''Accepts a primitive class, return the number of this primitive'''
        #parses the primitive values and generates the gl commands using vbo method (copying this information to gl memory) and generates a gl_list that then is stored in gl_primitives
        self.counter+=1
        self.gl_primitives[self.counter]=primitive
        return(self.counter)
    

    def remove_primitive(self,primitive_id):
        '''Give the primitive id returned by add_primitive, return True if an error occured'''
        #frees opengl memory for this vbo and gl_list data. Then erases the entry in self.gl_primitives
        if primitive_id in self.gl_primitives:
            del(self.gl_primitives[primitive_id])
            return(0)
        else:
            print "Primitive: ", primitive_id, " not present"
            return(1)

    def set_visibility(self,visibility):
        self.visibility=visibility

    def set_trans_rot_matrix(self,matrix):
        self.trans_rot_m=matrix

    def set_center_offset_matrix(self,matrix):
        self.center_offset_matrix=matrix
        self.gen_gl_list()

    def gen_gl_list(self):
        #This function should be run only when the object shape is generated or updated
        #generating objects gl lists
        for object_index in self.object_models:
            self.object_models[object_index].gen_gl_list()
        _gl.glNewList(self.gl_list,_gl.GL_COMPILE)
        #offset of object
        if self.center_offset_matrix != None:
            _gl.glPushMatrix()
            _gl.glMultMatrixf(self.center_offset_matrix.T)
        #inserting other objects
        for object_index in self.object_models:
            #print "Drawing object:", object_index
            object=self.object_models[object_index]
            if object.visibility:
                _gl.glPushMatrix()
                if object.scale != None:
                    _gl.glEnable(_gl.GL_NORMALIZE)
                _gl.glMultMatrixf(object.trans_rot_matrix.T)
                if object.scale != None:
                    _gl.glScale(object.scale[0],object.scale[1],object.scale[2])
                if object.material_specular_color != None:
                    _gl.glMaterialfv(_gl.GL_FRONT,_gl.GL_SPECULAR, object.material_specular_color)
                if object.material_shininess != None:
                    _gl.glMaterialf(_gl.GL_FRONT,_gl.GL_SHININESS, object.material_shininess)
                if object.material_ambient_color != None:
                    _gl.glMaterialfv(_gl.GL_FRONT,_gl.GL_AMBIENT, object.material_ambient_color)
                if object.material_diffuse_color != None:
                    _gl.glMaterialfv(_gl.GL_FRONT,_gl.GL_DIFFUSE, object.material_diffuse_color)
                _gl.glCallList(object.get_gl_list())
                if object.scale != None:
                    _gl.glDisable(_gl.GL_NORMALIZE)
                _gl.glPopMatrix()
        #iterate over all the primitives
        for primitive in self.gl_primitives:
            if self.gl_primitives[primitive].trans_rot_matrix:
                #we have to move the primitive
                _gl.glPushMatrix()
                _gl.glMultMatrixf(self.gl_primitives[primitive].trans_rot_matrix.T)
            if self.gl_primitives[primitive].vertices != None:
                _gl.glBegin(self.gl_primitives[primitive].type)
                if self.gl_primitives[primitive].material_specular_color:
                    _gl.glMaterialfv(_gl.GL_FRONT,_gl.GL_SPECULAR,self.gl_primitives[primitive].material_specular_color)
                if self.gl_primitives[primitive].material_shininess:
                    _gl.glMaterialf(_gl.GL_FRONT,_gl.GL_SHININESS,self.gl_primitives[primitive].material_shininess)
                if self.gl_primitives[primitive].material_ambient_color:
                    _gl.glMaterialfv(_gl.GL_FRONT,_gl.GL_AMBIENT,self.gl_primitives[primitive].material_ambient_color)
                if self.gl_primitives[primitive].material_diffuse_color:
                    _gl.glMaterialfv(_gl.GL_FRONT,_gl.GL_DIFFUSE,self.gl_primitives[primitive].material_diffuse_color)
                if self.gl_primitives[primitive].normals == None:
                    for i in xrange(self.gl_primitives[primitive].vertices.shape[0]):
                        _gl.glVertex3fv(self.gl_primitives[primitive].vertices[i])
                elif len(self.gl_primitives[primitive].normals.shape) == 1:
                    #print "Only one normal for all points" 
                    _gl.glNormal3fv(self.gl_primitives[primitive].normals)
                    for i in xrange(self.gl_primitives[primitive].vertices.shape[0]):
                        _gl.glVertex3fv(self.gl_primitives[primitive].vertices[i])
                elif self.gl_primitives[primitive].vertices.shape == self.gl_primitives[primitive].normals.shape:
                    for i in xrange(self.gl_primitives[primitive].vertices.shape[0]):
                        _gl.glNormal3fv(self.gl_primitives[primitive].normals[i])
                        _gl.glVertex3fv(self.gl_primitives[primitive].vertices[i])
                else:
                    print "Incorrect number of vertices/normals"
                    sys.exit()
                _gl.glEnd()
            if self.gl_primitives[primitive].trans_rot_matrix:
                _gl.glPopMatrix()
            if self.gl_primitives[primitive].gl_list:
                _gl.glCallList(self.gl_primitives[primitive].gl_list)
        if self.center_offset_matrix != None:
            _gl.glPopMatrix()
        _gl.glEndList()

    def get_gl_primitives(self):
        return(self.gl_primitives)

    def get_gl_list(self):
        return(self.gl_list)

class Light(object):
    LIGHTS=[_gl.GL_LIGHT0, _gl.GL_LIGHT1, _gl.GL_LIGHT2, _gl.GL_LIGHT3, _gl.GL_LIGHT4,_gl.GL_LIGHT5,_gl.GL_LIGHT6,_gl.GL_LIGHT7]
    def __init__(self,light_number):
        '''light number is _gl.GL_LIGHT*'''
        self.gl_list=_gl.glGenLists(1)
        self.diffuse_color=_n.array([1.,1.,1.0,1.0])
        self.specular_color=_n.array([1.,1.,1.,1.0])
        self.position=_n.array([0.,0.,0.,1.0])
        self.light_number=light_number
        self.activated=True
        self.gen_gl_list()

    def activate(self,state):
        self.activated=state

    def set_light_params(self,diffuse_color,specular_color, position):
        self.diffuse_color=diffuse_color
        self.specular_color=specular_color
        self.position=position
        self.gen_gl_list()

    def gen_gl_list(self):
        #This has to be run after some opengl initialization (display__init__)
        _gl.glNewList(self.gl_list,_gl.GL_COMPILE)
        _gl.glLightfv(self.light_number, _gl.GL_DIFFUSE, self.diffuse_color)
        _gl.glLightfv(self.light_number, _gl.GL_SPECULAR, self.specular_color)
        _gl.glLightfv(self.light_number, _gl.GL_POSITION, self.position)
        _gl.glEndList()

    def get_gl_list(self):
        return(self.gl_list)
    

class Scene(object):
    #it's composed of objects lights and an ambient light
    def __init__(self):
        self.objects={}
        self.lights={}
        self.ambient_light_color=_n.array([0.1,0.1,0.1,1.0])
        #self.camera #camera should be part of the scene
        self.counter=-1

    def set_ambient_light_color(self, ambient_light_color):
        self.ambient_light_color=ambient_light_color
        _gl.glLightModelfv(_gl.GL_LIGHT_MODEL_AMBIENT, self.ambient_light_color)

    def set_light_local_viewer(self, enable):
        if enable:
            _gl.glLightModeli(_gl.GL_LIGHT_MODEL_LOCAL_VIEWER,_gl.GL_TRUE)        
        else:
            _gl.glLightModeli(_gl.GL_LIGHT_MODEL_LOCAL_VIEWER,_gl.GL_FALSE)        

    def set_lighting(self,state):
        if state:
            self.set_ambient_light_color(self.ambient_light_color)
            _gl.glEnable(_gl.GL_LIGHTING)
        else:
            _gl.glDisable(_gl.GL_LIGHTING)

    def add_object(self,object_model):
        self.counter+=1
        object_model.gen_gl_list()
        self.objects[self.counter]=object_model
        return(self.counter)

    def remove_object(self,object_model_id):
        if object_model_id in self.objects:
            del(self.objects[object_model_id])

    def add_light(self,light):
        #TODO check that light number is not higher that the maximum allowed by open_gl.
        self.lights[light.light_number]=light
        light.gen_gl_list()

    def remove_light(self,number):
        if number in self.lights:
            del(self.lights[number])
        else:
            print "Light doesn't exist"

    def draw_scene(self):
        #inspect all self.objects, and execute only those that have visibility=True, inspect all lights and execute those ones activated. Execute the ambient light
        #camera
        #lights
        for light_index in self.lights:
            #print "Drawing light", light_index
            light=self.lights[light_index]
            if light.activated:
                _gl.glCallList(light.get_gl_list())
                _gl.glEnable(light.light_number)
        for object_index in self.objects:
            #print "Drawing object:", object_index
            object=self.objects[object_index]
            if object.visibility:
                _gl.glPushMatrix()
                if object.scale != None:
                    _gl.glEnable(_gl.GL_NORMALIZE)
                _gl.glMultMatrixf(object.trans_rot_matrix.T)
                if object.scale != None:
                    _gl.glScale(object.scale[0],object.scale[1],object.scale[2])
                if object.material_specular_color != None:
                    _gl.glMaterialfv(_gl.GL_FRONT,_gl.GL_SPECULAR, object.material_specular_color)
                if object.material_shininess != None:
                    _gl.glMaterialf(_gl.GL_FRONT,_gl.GL_SHININESS, object.material_shininess)
                if object.material_ambient_color != None:
                    _gl.glMaterialfv(_gl.GL_FRONT,_gl.GL_AMBIENT, object.material_ambient_color)
                if object.material_diffuse_color != None:
                    _gl.glMaterialfv(_gl.GL_FRONT,_gl.GL_DIFFUSE, object.material_diffuse_color)
                _gl.glCallList(object.get_gl_list())
                if object.scale != None:
                    _gl.glDisable(_gl.GL_NORMALIZE)
                _gl.glPopMatrix()

class Display(object):
    def __init__(self,camera,scene,res=(320,240),image_dump_dir=""):
        '''Takes a camera and a scene and creates a opengl display of resolution res'''
        self.camera=camera
        self.scene=scene
        self.size=res
        self.image_dump_dir=image_dump_dir
        if self.image_dump_dir!="":
            try:
                os.mkdir(self.image_dump_dir)
            except OSError:
                print "Directory already exists"
        _pygame.display.init()
        self.pg_surface=_pygame.display.set_mode(self.size,_pygame.OPENGL| _pygame.DOUBLEBUF)
        #_gl.glViewport(0,0,size[0],size[1])
        _gl.glMatrixMode(_gl.GL_PROJECTION)
        _gl.glLoadIdentity()
        _glu.gluPerspective(45,1.0*self.size[0]/self.size[1],0.1,1000.0)
        #_gl.glLoadIdentity()
        #_gl.glClearColor(0.0,0.0,0.0,0.0)
        #_gl.glClearDepth(1.0)
        #_gl.glEnable(GL_DEPTH_TEST)
        #_gl.glEnable(GL_ALPHA_TEST)
        #_gl.glHint(_gl.GL_PERSPECTIVE_CORRECTION_HINT,_gl.GL_NICEST)
        #_gl.glClear(_gl.GL_COLOR_BUFFER_BIT|_gl.GL_DEPTH_BUFFER_BIT)
        #_gl.glPointSize(5)
        self.grey=(0.50,0.50,0.50)
        self.black= (0,0,0)
        self.last_model_id=-1

        #draw planes correctly
        self.set_depth_test(True)

        #colors in planes
        _gl.glShadeModel(_gl.GL_SMOOTH)

        #lighting
        #setting material properties for all objects
        mat_specular=_n.array([1.0,1.,1.,1.])
        mat_shininess=50.
        _gl.glMaterialfv(_gl.GL_FRONT,_gl.GL_SPECULAR,mat_specular)
        _gl.glMaterialf(_gl.GL_FRONT,_gl.GL_SHININESS,mat_shininess)
        #lighting properties
        light_color=_n.array([1.,1.,1.,1.])
        ##light_model_ambient=_n.array([0.1,0.1,0.1,1.])
        #setting light0 parameters
        ##_gl.glLightfv(_gl.GL_LIGHT0, _gl.GL_DIFFUSE, light_color)
        ##_gl.glLightfv(_gl.GL_LIGHT0, _gl.GL_SPECULAR, light_color)
        #setting general lighting conditions. 
        ##_gl.glLightModelfv(_gl.GL_LIGHT_MODEL_AMBIENT, light_model_ambient)
        #better simulation for lighting
        ##_gl.glLightModeli(_gl.GL_LIGHT_MODEL_LOCAL_VIEWER,_gl.GL_TRUE)
        #enabling lighting
        ##_gl.glEnable(_gl.GL_LIGHTING)
        self.init_scene()

        #vertex arrays
        self.cube_vertices=_n.array([[-1.,-1.,-1],[-1,1,-1],[1,1,-1]],"f")
        self.cube_back_indices=_n.array([0,1,2])

        #buffer objets
        from OpenGL.arrays import vbo
        self.cube_vertices_vbo=vbo.VBO(self.cube_vertices,usage="GL_STATIC_DRAW")
        self.cube_vertices_vbo.bind()
        _gl.glBufferData(_gl.GL_ARRAY_BUFFER,self.cube_vertices_vbo.data,_gl.GL_STATIC_DRAW)
        #_gl.glBufferSubData(_gl.GL_ARRAY_BUFFER,0,None,cub_vertices)
        self.cube_back_indices_vbo=vbo.VBO(self.cube_back_indices,usage="GL_STATIC_DRAW", target="GL_ELEMENT_ARRAY_BUFFER")
        self.cube_back_indices_vbo.bind()
        _gl.glBufferData(_gl.GL_ELEMENT_ARRAY_BUFFER,self.cube_back_indices_vbo.data,_gl.GL_STATIC_DRAW)

        #gllists
        self.gl_list=_gl.glGenLists(1)
        _gl.glNewList(self.gl_list,_gl.GL_COMPILE)
        light_pos=_n.array([10.,10.,10.,1.0])
        _gl.glLightfv(_gl.GL_LIGHT0, _gl.GL_POSITION, light_pos)
        _gl.glEnable(_gl.GL_LIGHT0)
        _gl.glEndList()

        #For image capture
        self.img_cnt=0
        self.init_time=time.time()
        
        
    def set_scene(self,scene):
        self.scene=scene
        self_init_scene()

    def init_scene(self):
        self.scene.set_light_local_viewer(True)
        self.scene.set_lighting(True)

    def set_depth_test(self,state):
        if state:
            _gl.glEnable(_gl.GL_DEPTH_TEST)
        else:
            _gl.glDisable(_gl.GL_DEPTH_TEST)

    def set_camera(self,camera):
        self.camera=camera

    def update(self):
        '''Updates the screen using the scene information and the camera'''
        #clear screen
        _gl.glClear(_gl.GL_COLOR_BUFFER_BIT| _gl.GL_DEPTH_BUFFER_BIT)

        #camera
        _gl.glMatrixMode(_gl.GL_MODELVIEW)
        _gl.glLoadMatrixf(self.camera.get_camera_matrix().T)

        self.scene.draw_scene()
        
        #general color
        ##mat_specular=_n.array([1.0,1.,1.,1.])
        ##_gl.glMaterialfv(_gl.GL_FRONT,_gl.GL_SPECULAR,mat_specular)
        ##mat_ambient_diffuse_color=_n.array([1.0,1.,1.,1.])
        ##_gl.glMaterialfv(_gl.GL_FRONT,_gl.GL_AMBIENT_AND_DIFFUSE,mat_ambient_diffuse_color)

        #update!
        _pygame.display.flip()

        #pixel_data=_gl.glReadPixels(0.,0.,self.size[0],self.size[1],_gl.GL_RGB,type=_gl.GL_UNSIGNED_INT)
        if self.image_dump_dir!="":
            filename=self.image_dump_dir+"capture-"+str(self.init_time)+"-%07d.png" % (self.img_cnt)
            _pygame.image.save(self.pg_surface,filename)
            self.img_cnt+=1

class Camera_matrix(object):
    #camera model: Its a sphere:
    # We control with the mouse the azimuth, the inclination and the radius (scrolling). With ctrl we control the center of the sphere X, Y and Z with respect to the camera coordinates. And with Alt and scroll we control the rotation of the camera around its normal.
    def __init__(self):
        self.center_rot_frame=_n.identity(4)
        self.delta_pos=_n.array([0.,0.,0.])
        self.altitud=0.
        self.azimuth=0.
        self.radius=-15.
        self.normal_angle=0.
    def change_center(self,delta_pos):
        '''Changes the center of the sphere by delta pos'''
        self.delta_pos=delta_pos
    def change_sphere(self,azimuth,altitud,radius):
        '''Changes the sphere parameters'''
        self.azimuth=azimuth
        self.altitud=altitud
        self.radius+=radius

    def rotate_normal(self,angle):
        '''Rotates the camera with respect to its own normal'''
        self.normal_angle=angle

    def get_frame(self):
        '''Gets a new frame'''
        self.delta_pos_m=_n.identity(4)
        self.delta_pos_m[:,3][:3]=self.delta_pos
        #print "delta pos: ", self.delta_pos
        rot_m_x=_n.identity(4)
        rot_m_y=_n.identity(4)
        rot_m_z=_n.identity(4)
        rot_m_x[:3,:3]=rotx(self.altitud)
        rot_m_y[:3,:3]=roty(self.azimuth)
        rot_m_z[:3,:3]=rotz(self.normal_angle)
        rot_m=_n.dot(_n.dot(rot_m_x,rot_m_y),rot_m_z)
        self.center_rot_frame=_n.dot(_n.dot(rot_m,self.delta_pos_m),self.center_rot_frame)
        radius_m=_n.identity(4)
        radius_m[2,3]=self.radius
        frame=_n.dot(radius_m,self.center_rot_frame)
        self.delta_pos=_n.array([0.,0.,0.])
        self.altitud=0.
        self.azimuth=0.
        self.normal_angle=0.
        #print "Matrix: ", frame
        return(frame)
        
class Camera(object):
    def __init__(self):
        self.camera_matrix=Camera_matrix()
        self.buttons=_n.array([False]*5)
        self.rot_inc=1*_n.pi/180.0
        self.xyz_inc=0.1
        
    def update(self,event):
        if event.type == _pygame.MOUSEMOTION:
            if event.buttons == (1,0,0):
                self.camera_matrix.change_sphere(self.rot_inc*event.rel[0],self.rot_inc*event.rel[1],0.)
            if event.buttons == (0,0,1):
                self.camera_matrix.change_center(_n.array([self.xyz_inc*event.rel[0],-self.xyz_inc*event.rel[1],0.]))
            return()
        if event.type == _pygame.MOUSEBUTTONDOWN:
            if event.button <= 5:
                self.buttons[event.button-1]=True
        elif event.type == _pygame.MOUSEBUTTONUP:
            if event.button <= 5:
                self.buttons[event.button-1]=False
        else:
            return()
        if self.buttons[0] and not self.buttons[2] and self.buttons[3]:
            self.camera_matrix.rotate_normal(self.rot_inc*4)
        if self.buttons[0] and not self.buttons[2] and self.buttons[4]:
            self.camera_matrix.rotate_normal(-self.rot_inc*4)
        if self.buttons[2] and not self.buttons[0] and self.buttons[3]:
            self.camera_matrix.change_center(_n.array([0.,0.,self.xyz_inc*4]))
        if self.buttons[2] and not self.buttons[0] and self.buttons[4]:
            self.camera_matrix.change_center(_n.array([0.,0.,-self.xyz_inc*4]))
        if _n.invert(self.buttons[:3]).all():
            #normal zoom (radius)
            if self.buttons[3]:
                self.camera_matrix.change_sphere(0.,0.,self.xyz_inc*4)
            if self.buttons[4]:
                self.camera_matrix.change_sphere(0.,0.,-self.xyz_inc*4)

    def get_camera_matrix(self):
        return(self.camera_matrix.get_frame())

