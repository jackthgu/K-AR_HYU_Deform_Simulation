#!BPY

"""
Name: 'Taesoo import model objects (.obj)...'
Blender: 249
Group: 'Object'
"""

__author__= "Taesoo Kwon"
__version__= "0.01"

__bpydoc__= """\
This script imports all objects in the path to Blender.

Usage:
Run this script from "scripts" menu 
Note, This loads mesh objects and materials only, nurbs and curves are not supported.
"""
#import Blender
import bpy
import os,sys
#sys.path.append(os.path.join(os.path.dirname(__file__), "lib"))
sys.path.append(os.path.dirname(__file__))
#list.append(sys.path, '/home/taesoo/taesooLib/work')
#print sys.path

#import import_obj2 # prerequiste : execute l blender_install
import io_scene_obj.import_obj # prerequiste : execute l blender_install
import blenderConfig 

if __name__=='__main__':
	bpy.context.scene.objects.unlink(bpy.data.objects['Cube'])
	files= [ f for f in os.listdir(blenderConfig.path) if f.lower().endswith('.obj') ]
	for file in files:
		if file.find('backup')==-1 and file.find('100.obj')==-1:
			io_scene_obj.import_obj.load('op', bpy.context,blenderConfig.path+'/'+file)
			bpy.context.selected_objects[0].name=file
