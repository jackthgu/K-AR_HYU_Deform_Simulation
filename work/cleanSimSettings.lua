targets={'walk3','backflip', 'roundoff2', 'straddle', 'popa2', 'justinStraightRun_nocart'}
require('config')


files={}

lines={}
for i, target in ipairs(targets) do
	files[i]=string.lines(util.readFile('cp_all'..target))

	for ii,l in ipairs(files[i]) do
		if not lines[l] then
			lines[l]=1
		else
			lines[l]=lines[l]+1
		end
	end
end

for ii, l in pairsByKeys(lines)do 
	print(ii,l)
end

