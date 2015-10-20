# imports
import shlex

# constants and variables
dataInfo = 5000
pastPose = [0,0,0]
nodeID = 0

# filenames
inDeadReckon = "deadReckoning.dat"
inMeasurement = "measurements.dat"
outG2O = "victoriaPark.g2o"

# file objects
fdr = open(inDeadReckon, 'r')
fm = open(inMeasurement, 'r')
fg2o = open(outG2O, 'w')

# getting odometry
header_line = next(fdr)
for line in fdr:
    # split string
    lineWords = shlex.split(line)
    currentPose = [float(lineWords[1]), float(lineWords[2]), float(lineWords[3])]
    #dx = currentPose[0] - pastPose[0]
    #dy = currentPose[1] - pastPose[1]
    #dt = currentPose[2] - pastPose[2]
    #pastPose = currentPose 
    #dx = currentPose[0]
    #dy = currentPose[1]
    #dt = currentPose[2]
    #fg2o.write("EDGE_SE2 " + str(nodeID) + " " + str(nodeID+1) + " " + str(dx) + " " 
    #    + str(dy) + " " + str(dt) + " " + str(dataInfo) + " 0 0 " + str(dataInfo) + " 0 " + str(dataInfo) + "\n")
    fg2o.write("VERTEX_SE2 " + str(nodeID) + " " + lineWords[1] + " " + lineWords[2] + " " + lineWords[3] + "\n")
    nodeID = nodeID + 1

# setting fix
fg2o.write("FIX 0\n")
