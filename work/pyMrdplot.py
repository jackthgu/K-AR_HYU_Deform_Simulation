from numpy import *
import matplotlib
matplotlib.use('Agg')
from pylab import *
import sys
import os 
if __name__ == "__main__":

	sub_height=3
	if len(sys.argv)>1:
		fn=sys.argv[1]
		print('input file name:' ,fn)
		mrdplot=__import__(fn)
		data=matrix(mrdplot.data)
		nsubFig=0
		subFigs={}
		tagsToSubfigIndex={}
		for i in range(len(mrdplot.data[0])):
			if mrdplot.tags[i]!="_":
				if tagsToSubfigIndex.has_key(mrdplot.tags[i]):
					si=tagsToSubfigIndex[mrdplot.tags[i]]
					subFigs[si]['cols'][len(subFigs[si]['cols'])]=i
					subFigs[si]['legends'][len(subFigs[si]['legends'])]=mrdplot.names[i]
				else:
					subFigs[nsubFig]={'tagname':mrdplot.tags[i], 'cols':{0:i}}
					subFigs[nsubFig]['legends']={0:mrdplot.names[i]}
					tagsToSubfigIndex[mrdplot.tags[i]]=nsubFig
					nsubFig=nsubFig+1
			else:
				subFigs[nsubFig]={'tagname':mrdplot.names[i], 'cols':{0:i}}
				tagsToSubfigIndex[mrdplot.tags[i]]=nsubFig
				nsubFig=nsubFig+1



		print("# of subfigures: ",nsubFig, subFigs)




		#print mrdplot.names
		nsubfig=len(subFigs)
		maxSubFig=int(sys.argv[2])
		for k in range(0, nsubfig, maxSubFig):
			#print nsubfig
			print('nsubfig: ', sub_height*nsubfig)
			figure(1, figsize=(12,sub_height*min(maxSubFig,nsubfig)))
			grid(True)
			for i in range(k,min(k+maxSubFig,nsubfig)):
				print('maxsubfig: ', maxSubFig,i+1-k, k, min(k+maxSubFig,nsubFig))
				subplot(min(maxSubFig,nsubfig),1,i+1-k)
				t=arange(shape(data)[0])
				#print(t)
				for j in range(len(subFigs[i]['cols'])):
					plot(t,data[:,subFigs[i]['cols'][j]])
				if subFigs[i].has_key('legends'):
					legend(subFigs[i]['legends'].values())
				ylabel(subFigs[i]['tagname'])
			print('output file name:', fn+str(k)+'.png')
			savefig(fn+str(k)+'.png')
			close(1)
			#os.system('gnome-open '+fn+str(k)+'.png')
	else:
		print("Usage: write data.py and run pyMrdplot.py as follows:")
		print(' python pyMrdplot.py data 40')
