import sys
from subprocess import call

# Handling Error
import os

if len(sys.argv) == 1:
	print "Usage : objmultiply2.py [Filename.obj]"
	exit()

objFile = open(sys.argv[1], "r")
obj2File = open(sys.argv[1].replace(".obj", "by2.obj"), "w")

print "Name of the input file: ", objFile.name
print "Name of the output file: ", obj2File.name

vertexCount = 0
faceCount = 0
vertices = []
faces = []
for line in objFile:
	lineSpace = line.replace("//", "/")
	lineSpace = lineSpace.replace("/", " ")
	lineSpace = lineSpace.replace("\n", " ")
	lineElements = lineSpace.split(" ")
	if lineElements[0] == "v": # vertices
		obj2File.write(lineElements[0] +" "+ str(float(lineElements[1])/2.0) +" "+ str(float(lineElements[2])/2.0) +" "+ str(float(lineElements[3])/2.0)+"\n")
		print lineElements[0] +" "+ str(float(lineElements[1])/2.0) +" "+ str(float(lineElements[2])/2.0) +" "+ str(float(lineElements[3])/2.0)
	elif lineElements[0] == "l": # Remove lines - error in taesooLib
		pass
		# print line
	else:
		obj2File.write(line)
		# print line
