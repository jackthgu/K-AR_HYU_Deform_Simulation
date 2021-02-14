import sys
from subprocess import call

# Handling Error
import os

if len(sys.argv) == 1:
	print "Usage : objtopoly.py [Filename.obj]"
	exit()

objFile = open(sys.argv[1], "r")
outputFile = open(sys.argv[1].replace(".obj", ".poly"), "w")

print "Name of the input file: ", objFile.name
print "Name of the output file: ", outputFile.name

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
		obj2File.write(lineElements[0] +" "+ str(float(lineElements[1])*2) +" "+ str(float(lineElements[2])*2) +" "+ str(float(lineElements[3])*2)+"\n")
		print lineElements[0] +" "+ str(float(lineElements[1])*2) +" "+ str(float(lineElements[2])*2) +" "+ str(float(lineElements[3])*2)
	else:
		obj2File.write(line)
		print line
