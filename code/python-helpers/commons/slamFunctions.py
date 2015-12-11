# imports
from slamData import slamData
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os
import errno
import math

# functions
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
   
def plotGuess (gtFilename, guessFilename, figFilename):
    # get variables from file
    gtData = slamData(gtFilename)
    guessData = slamData(guessFilename)
    
    lw = 3
    ms = 8
    
    if gtData is not None:
        gtRobPlt, = plt.plot(gtData.poseX, gtData.poseY, '.-', color = '#bbbbf9', linewidth = lw, markersize = ms, label='GT robot path')
    lanPlt, = plt.plot(guessData.landmarkX, guessData.landmarkY, 'r+', linewidth = lw, markersize = ms, label='landmarks')
    robPlt, = plt.plot(guessData.poseX, guessData.poseY, 'b-', linewidth = lw, markersize = ms, label='robot path')
    if gtData is not None:
        gtLanPlt, = plt.plot(gtData.landmarkX, gtData.landmarkY, '.', color = '#800000', linewidth = lw, markersize = ms, label='GT landmarks')
    
    ax = plt.axes()
    ax.grid(True)
    ax.set_title("Initial Guess")
    ax.relim()
    ax.autoscale_view()
    ax.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.savefig(figFilename + ".pdf", bbox_inches='tight')
    
def plotResults(gtFilename, guessFilename, optFilename, figFilename, suffix):
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
    #plt.savefig(figFilename + ".png", bbox_inches='tight')
    plt.savefig(figFilename + suffix + ".pdf", bbox_inches='tight')
    
    # plot path error
    pathPlot(gtData, optData, figFilename, suffix)
    
def makeSubplot(ax, gtData, slamData, title, useLegend):
    lw = 2
    ms = 8
    if gtData is not None:
        gtRobPlt, = ax.plot(gtData.poseX, gtData.poseY, '.-', color = '#bbbbf9', linewidth = lw, markersize = ms, label='GT robot path')
    lanPlt, = ax.plot(slamData.landmarkX, slamData.landmarkY, 'r+', linewidth = lw, markersize = ms, label='landmarks')
    robPlt, = ax.plot(slamData.poseX, slamData.poseY, 'b-', linewidth = lw, markersize = ms, label='robot path')
    if gtData is not None:
        gtLanPlt, = ax.plot(gtData.landmarkX, gtData.landmarkY, '.', color = '#800000', linewidth = lw, markersize = ms, label='GT landmarks')
    ax.grid(True)
    ax.set_title(title)
    ax.relim()
    ax.autoscale_view()
    if useLegend:
        ax.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
        
def pathPlot(gtData, optData, figFilename, suffix):
    lw = 3
    pathError = []
    firstError = math.sqrt((gtData.poseX[0] - optData.poseX[0])**2 + 
    (gtData.poseY[0] - optData.poseY[0])**2)
    pathError.append(firstError)
    
    for i in range(1, len(gtData.poseX)):
        error = math.sqrt((gtData.poseX[i] - optData.poseX[i])**2 + 
        (gtData.poseY[i] - optData.poseY[i])**2)
        pathError.append(error + pathError[i-1]);
    
    for i in range(len(pathError)):
        pathError[i] = pathError[i] / (i+1);
    
    f = plt.figure();
    plt.plot(range(1,len(pathError)+1), pathError, linewidth = lw)
    plt.grid(True)
    f.suptitle("Path Error")
    plt.xlabel("Timestep")
    plt.ylabel("Normalized error")
    plt.xlim([1, len(pathError)+1])
    plt.ylim([0, max(pathError)])
    #plt.savefig(figFilename + "_path.png", bbox_inches='tight')
    plt.savefig(figFilename + suffix + "_path.pdf", bbox_inches='tight')
    
def makeRealPlots (guessPath, optPath, figPath, suffix):
    # get data from file
    guessData = slamData(guessPath)
    optData = slamData(optPath)
    f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
    makeSubplot(ax1, None, guessData, "Initial Guess", False)
    makeSubplot(ax2, None, optData, "After Solver", True)
    
    # make figure
    plt.savefig(figPath + suffix + ".pdf", bbox_inches='tight')
    #plt.show()
