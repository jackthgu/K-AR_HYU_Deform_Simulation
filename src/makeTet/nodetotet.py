import sys

if len(sys.argv) == 1:
	print "Usage : nodetotet.py [Filename.1]"
	exit()

nodeFile = open(sys.argv[1] + ".node", "r")
eleFile = open(sys.argv[1] + ".ele", "r")
tetFile = open(sys.argv[1] + ".tet", "w")

print "Name of the input file: ", nodeFile.name, " & ", eleFile.name
print "Name of the output file: ", tetFile.name

tetCount = 0
vertices = []
tets = []

# vertex
firstline = nodeFile.readline()
firstlineElements = firstline.split(" ")
vertexCount = firstlineElements[0]
count = 1
for line in nodeFile:
	line = line.replace("\n", " ")
	lineElements = line.split()

	print count, lineElements[1], " ",lineElements[2], " ",lineElements[3], "\n"

	if lineElements[0] == str(count):  # vertices
		print count, lineElements[1], " ",lineElements[2], " ",lineElements[3], "\n"
		vertices += [lineElements[1], lineElements[2], lineElements[3]]

	count += 1

# elements
firstline = eleFile.readline()
firstlineElements = firstline.split(" ")
eleCount = firstlineElements[0]
count = 1
for line in eleFile:
	line = line.replace("\n", " ")
	lineElements = line.split()

	if lineElements[0] == str(count):  # elements
		print count, lineElements[1], " ",lineElements[2], " ",lineElements[3], " ",lineElements[4], "\n"
		# .ele file starts from 1, but .tet file starts from 0
		tets += [int(lineElements[1])-1, int(lineElements[2])-1, int(lineElements[3])-1, int(lineElements[4])-1]

	count += 1

print "vertexCount : ", len(vertices)/3, "\ntetCount : ", len(tets)/4


tetFile.write(str(len(vertices)/3) + " " + str(len(tets)/4) + "\n")
# print vertex
for vertex in range(len(vertices)/3):
	tetFile.write(vertices[vertex * 3] + " " + vertices[vertex * 3 + 1] + " " + vertices[vertex * 3 + 2] + "\n")

# print eles
for tet in range(len(tets) / 4):
	tetFile.write(str(tets[tet * 4]) + " " + str(tets[tet * 4 + 1]) + " " + str(tets[tet * 4 + 2]) + " " + str(tets[tet * 4 + 3]) + "\n")
