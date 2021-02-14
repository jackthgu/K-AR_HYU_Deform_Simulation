import sys
from subprocess import call

# Handling Error
import os

if len(sys.argv) == 1:
    print ("Usage : objtopoly.py [Filename.obj]")
    exit()

objFile = open(sys.argv[1], "r")
polyFile = open(sys.argv[1].replace(".obj", ".poly"), "w")

print ("Name of the input file: ", objFile.name)
print ("Name of the output file: ", polyFile.name)

vertexCount = 0
faceCount = 0
vertices = []
faces = []
for line in objFile:
    line = line.replace("  ", " ")
    line = line.replace("//", "/")
    line = line.replace("/", " ")
    line = line.replace("\n", " ")
    lineElements = line.split(" ")

    if lineElements[0] == "v": # vertices
        vertices += [lineElements[1], lineElements[2], lineElements[3]]
        print (vertexCount+1, vertices[vertexCount*3], vertices[vertexCount*3+1], vertices[vertexCount*3+2])
        vertexCount += 1

    elif lineElements[0] == "f": # faces
        if len(lineElements) > 9:
            faces += [lineElements[1], lineElements[4], lineElements[7]] # TODO: Make it more GENERAL
        elif len(lineElements) > 6:
            faces += [lineElements[1], lineElements[3], lineElements[5]] # TODO: Make it more GENERAL
        else:
            faces += [lineElements[1], lineElements[2], lineElements[3]] # TODO: Make it more GENERAL
        print ("1\n3", faces[faceCount*3], faces[faceCount*3+1], faces[faceCount*3+2])
        faceCount += 1

print ("vertexCount : ", vertexCount, "\nfaceCount : ", faceCount)
    #polyFile.write(line)

#polyFile.write("\n\n")
# print vertex
count = 0
polyFile.write(str(vertexCount) + " 3 0 0\n")
for vertex in range(len(vertices)/3):
    polyFile.write(str(count+1) +" "+vertices[count*3] +" "+vertices[count*3+1]+" "+vertices[count*3+2] + "\n")
    count+=1
polyFile.write("\n\n")

# print facets
polyFile.write(str(faceCount) + " 1\n")
for face in range(len(faces)/3):
    polyFile.write("1\n3"+ " "+faces[face*3]+ " "+faces[face*3+1]+ " "+faces[face*3+2]+ "\n")
