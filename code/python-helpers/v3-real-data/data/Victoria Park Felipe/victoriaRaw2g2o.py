# imports
import shlex
import math

# constants and variables
dataInfo = 5000
pastPose = [0,0,0]
poseID = 0

# filenames
inDeadReckon = "deadReckoning.dat"
inMeasurement = "measurements.dat"
outG2O = "victoria.g2o"

# file objects
fdr = open(inDeadReckon, 'r')
fm = open(inMeasurement, 'r')
fg2o = open(outG2O, 'w')

# get poses
header_line = next(fdr)
for line in fdr:
    # split string
    lineWords = shlex.split(line)
    currentPose = [float(lineWords[1]), float(lineWords[2]), float(lineWords[3])]
    fg2o.write("VERTEX_SE2 " + str(poseID) + " " + lineWords[1] + " " + lineWords[2] + " " + lineWords[3] + "\n")
    poseID = poseID + 1

# setting fix
landID = poseID
poseID = 0
fg2o.write("FIX " + str(poseID) + "\n")

# reset fdr
fdr.seek(0)

# get odometry and measurements
header_line = next(fdr)
header_line = next(fm)
#for odomLine in fdr:
odomLine = fdr.next()
measLine = fm.next()
while odomLine:
    # get odometry
    #print odomLine
    odomWords = shlex.split(odomLine)
    odomTime = float(odomWords[0])
    currentPose = [float(odomWords[1]), float(odomWords[2]), float(odomWords[3])]
    dx = currentPose[0] - pastPose[0]
    dy = currentPose[1] - pastPose[1]
    dt = ((currentPose[2] - pastPose[2] + math.pi) % (2*math.pi)) - math.pi
    fg2o.write("EDGE_SE2 " + str(poseID) + " " + str(poseID+1) + " " + str(dx) + " " +
        str(dy) + " " + str(dt) + " " + str(dataInfo) + " 0 0 " + str(dataInfo) + " 0 " + str(dataInfo) + "\n")
    pastPose = currentPose 
    
    # get measurement time
    while measLine:
        #print measLine
        measWords = shlex.split(measLine)
        measTime = float(measWords[0])
        if measTime <= odomTime:
            px = float(odomWords[1])
            py = float(odomWords[2])
            pt = float(odomWords[3])
            mr = float(measWords[1])
            mt = float(measWords[2])
            lx = mr*math.cos(pt+mt) + px
            ly = mr*math.sin(pt+mt) + py
            fg2o.write("EDGE_SE2_XY " + str(poseID) + " " + str(landID) + " " + str(lx) +
                " " + str(ly) + " " + str(dataInfo) + " 0 " + str(dataInfo) + "\n")
            landID = landID + 1
            try:
                measLine = fm.next()
            except(StopIteration):
                break
        else:
            break
    poseID  = poseID + 1 
    try:
        odomLine = fdr.next()
    except(StopIteration):
        break 