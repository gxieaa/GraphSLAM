# imports
import shlex
import math

# constants and variables
dataInfo = 5000
pastPose = [0,0,0]
poseID = 0

# filenames
inDeadReckon = "deadReckoning.dat"
inMeasurement = "measurement.dat"
outG2O = "ohiggins.g2o"

# file objects
fdr = open(inDeadReckon, 'r')
fm = open(inMeasurement, 'r')
fg2o = open(outG2O, 'w')

# get poses
header_line = next(fdr)
for line in fdr:
    # split string
    lineWords = shlex.split(line)
    currentPose = [float(lineWords[2]), float(lineWords[3]), float(lineWords[4])]
    fg2o.write("VERTEX_SE2 " + str(poseID) + " " + lineWords[2] + " " + lineWords[3] + " " + lineWords[4] + "\n")
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
measLine = fm.next()
odomLine = fdr.next()
odomWords = shlex.split(odomLine)
pastPose = [float(odomWords[2]), float(odomWords[3]), float(odomWords[4])]
odomLine = fdr.next()
while True:
    # get odometry
    #print odomLine
    odomWords = shlex.split(odomLine)
    odomTime = 1000000000*int(odomWords[0]) + int(odomWords[1])
    currentPose = [float(odomWords[2]), float(odomWords[3]), float(odomWords[4])]
    dx = currentPose[0] - pastPose[0]
    dy = currentPose[1] - pastPose[1]
    dt = ((currentPose[2] - pastPose[2] + math.pi) % (2*math.pi)) - math.pi
    fg2o.write("EDGE_SE2 " + str(poseID) + " " + str(poseID+1) + " " + str(dx) + " " +
        str(dy) + " " + str(dt) + " " + str(dataInfo) + " 0 0 " + str(dataInfo) + " 0 " + str(dataInfo) + "\n")
    pastPose = currentPose 
    
    # get measurement time
    while True:
        #print measLine
        measWords = shlex.split(measLine)
        measTime = 1000000000*int(measWords[0]) + int(measWords[1])
        if measTime <= odomTime:
            px = float(odomWords[2])
            py = float(odomWords[3])
            pt = float(odomWords[4])
            mr = float(measWords[2])
            mt = float(measWords[3])
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
