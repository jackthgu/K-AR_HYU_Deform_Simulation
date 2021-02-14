import sys
from subprocess import call

# Handling Error
import os

if len(sys.argv) == 1:
    print "Usage : objtopoly.py [Filename.obj]"
    exit()

objFile = open(sys.argv[1], "r")
xyzFile = open(sys.argv[1].replace(".obj", ".xyz"), "w")

print "Name of the input file: ", objFile.name
print "Name of the output file: ", xyzFile.name

vertexCount = 0
faceCount = 0
vertices = []
faces = []
for line in objFile:
    line = line.replace("//", "/")
    line = line.replace("/", " ")
    line = line.replace("\n", " ")
    lineElements = line.split(" ")

    if lineElements[0] == "v": # vertices
        vertices += [lineElements[1], lineElements[2], lineElements[3]]
        print vertexCount+1, vertices[vertexCount*3], vertices[vertexCount*3+1], vertices[vertexCount*3+2]
        vertexCount += 1

    elif lineElements[0] == "f": # faces
        faces += [lineElements[1], lineElements[2], lineElements[3]] # TODO: Make it more GENERAL
        print "1\n3", faces[faceCount*3], faces[faceCount*3+1], faces[faceCount*3+2]
        faceCount += 1

print "vertexCount : ", vertexCount, "\nfaceCount : ", faceCount
    #polyFile.write(line)

#polyFile.write("\n\n")
# print vertex
count = 0
for vertex in range(len(vertices)/3):
    xyzFile.write(vertices[count * 3] + " " + vertices[count * 3 + 1] + " " + vertices[count * 3 + 2] + "\n")
    count+=1