# imports
import shlex
import string

# clam data class
class slamData():
 
    def __init__(self, dataFilename):
        self.landmarkX = []
        self.landmarkY = []
        self.poseX = []
        self.poseY = []
        self.getDataFromFile(dataFilename)
        
    def getDataFromFile(self, dataFilename):
        f = open(dataFilename, 'r')
        # get data loop
        for line in f:
            # split string
            lineWords = shlex.split(line)
            if string.find(lineWords[0], "VERTEX_SE2") != -1:
                # get robot pose
                self.poseX.append(float(lineWords[2]))
                self.poseY.append(float(lineWords[3]))
            elif string.find(lineWords[0], "VERTEX_XY") != -1:
                # get landmark position
                self.landmarkX.append(float(lineWords[2]))
                self.landmarkY.append(float(lineWords[3]))
        f.close()
