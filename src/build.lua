#!lua
EXE='flexiblebody'

local f=io.open('../make.lua','r')
if f then
	f:close()
	dofile('../make.lua')
else
	dofile('../../make.lua')
end
