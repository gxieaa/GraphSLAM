# imports
from slamData import slamData
import matplotlib.pyplot as plt
import os
import errno

# functions
def makeDirs(dirNames):
    for dirName in dirNames:
        try:
            os.makedirs(dirName)
        except OSError as exception:
            if exception.errno != errno.EEXIST:
                raise   

def genFilename(prefix, paramNames, params, sufix):
    s = prefix
    for i in range(len(params)):
        s = s + "_" + str(paramNames[i]) + "_" + str(params[i])
    return s + sufix
     
def fixNode(filename, node):
    f = open(filename, 'a')
    f.write("FIX " + str(node))
    f.close()

def getInitialGuess(gtFilename, guessFilename):
    fGt = open(gtFilename, 'r')
    fGuess = open(guessFilename, 'w')

    for line in fGt:
        if line[0:4] == "EDGE" or line[0:3] == "FIX":
            fGuess.write(line)

    fGt.close()
    fGuess.close()
    
def plotResults(gtFilename, guessFilename, optFilename, figFilename, xlim, ylim):
    # get variables from file
    gtData = slamData(gtFilename)
    guessData = slamData(guessFilename)
    optData = slamData(optFilename)
    
    # create figure
    f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
    makeSubplot(ax1, gtData, guessData, "Initial Guess", False)
    makeSubplot(ax2, gtData, optData, "After Solver", True)
    #plt.show()
    # print figure
    plt.xlim(xlim)
    plt.ylim(ylim)
    plt.savefig(figFilename + ".png", bbox_inches='tight')
    plt.savefig(figFilename + ".pdf", bbox_inches='tight')
    
def makeSubplot(ax, gtData, slamData, title, useLegend):
    lw = 1
    ms = 4
    gtLanPlt, = ax.plot(gtData.landmarkX, gtData.landmarkY, '.', color = '#800000', linewidth = lw, markersize = ms, label='GT landmarks')
    gtRobPlt, = ax.plot(gtData.poseX, gtData.poseY, '.-', color = '#bbbbf9', linewidth = lw, markersize = ms, label='GT robot path')
    lanPlt, = ax.plot(slamData.landmarkX, slamData.landmarkY, 'r.', linewidth = lw, markersize = ms, label='landmarks')
    robPlt, = ax.plot(slamData.poseX, slamData.poseY, 'b.-', linewidth = lw, markersize = ms, label='robot path')
    ax.grid(True)
    ax.set_title(title)
    if useLegend:
        ax.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
