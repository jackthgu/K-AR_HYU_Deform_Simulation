import sys
from subprocess import call

# Handling Error
import os

if len(sys.argv) != 2:
    print "Usage : offtoobj.py [Filename.off]"
    exit()

offFile = open(sys.argv[1], "r")
objFile = open(sys.argv[1].replace(".off", ".obj"), "w")

print "Name of the input file: ", offFile.name
print "Name of the output file: ", objFile.name

firstline = offFile.readline() #OFF
firstline = offFile.readline()
firstlineElements = firstline.split(" ")


vertexCount = firstlineElements[0]
faceCount = firstlineElements[1]
vertices = []
faces = []

count = 0
fcount = 0

for line in offFile:
    lineElements = line.split()

    if len(lineElements) == 3: # vertices
        vertices += [lineElements[0], lineElements[1], lineElements[2]]
        print count, vertices[count*3], vertices[count*3+1], vertices[count*3+2]
        count += 1

    elif len(lineElements) == 4: # faces
        faces += [int(lineElements[1])+1, int(lineElements[2])+1, int(lineElements[3])+1] # TODO: Make it more GENERAL
        print "3", faces[fcount*3], faces[fcount*3+1], faces[fcount*3+2]
        fcount += 1

print "vertexCount : ", vertexCount, "\nfaceCount : ", faceCount

#polyFile.write("\n\n")
# print vertex
count = 0
for vertex in range(len(vertices)/3):
    objFile.write("v " + vertices[count * 3] + " " + vertices[count * 3 + 1] + " " + vertices[count * 3 + 2] + "\n")
    count+=1

# print facets
for face in range(len(faces)/3):
    objFile.write("f " + str(faces[face * 3]) + " " + str(faces[face * 3 + 1]) + " " + str(faces[face * 3 + 2]) + "\n")
