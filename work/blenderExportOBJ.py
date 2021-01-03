#!BPY
 
"""
Name: 'Taesoo export selected model objects (.obj)...'
Blender: 249
Group: 'Object'
"""

__author__= "Taesoo Kwon"
__version__= "0.01"

__bpydoc__= """\
This script export all objects in the path to Blender.

Usage:
Run this script from "scripts" menu 
Note, This loads mesh objects and materials only, nurbs and curves are not supported.
"""
#import Blender
import os,sys
#sys.path.append(os.path.join(os.path.dirname(__file__), "lib"))
sys.path.append(os.path.dirname(__file__))
import blenderImportOBJ
import blenderConfig 
import bpy
import io_scene_obj.export_obj

if __name__=='__main__':
	
	ctx = bpy.context
	for obj in ctx.selected_objects:
		io_scene_obj.export_obj.write_file(blenderConfig.path+obj.name, [obj], ctx.scene)
