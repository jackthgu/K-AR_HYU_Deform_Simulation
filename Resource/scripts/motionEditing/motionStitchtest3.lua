a=Motion(1,150,200)
b=Motion(1,250,300)
e=stitch(a,b, "stitchUsingRoot2")
e:show()
f=stitch(a,b, "concat")
f:show("green")
