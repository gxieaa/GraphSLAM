# imports
import shlex
import string
import numpy

dataFilename = "simple_corr2.g2o"
f1 = open(dataFilename)
outputFilename = dataFilename[0:dataFilename.index('.')] + "_noise.g2o"
f2 = open(outputFilename, 'w')

# get data loop
for line in f1:
    # split string
    lw = shlex.split(line)
    # if line is edge SE2
    if lw[0] == "EDGE_SE2":
        # add noise
        px_noise = float(lw[3]) + numpy.random.normal(0, 1/float(lw[6]))
        py_noise = float(lw[4]) + numpy.random.normal(0, 1/float(lw[9]))
        pa_noise = float(lw[5]) + numpy.random.normal(0, 1/float(lw[11]))
        f2.write(lw[0]+" "+lw[1]+" "+lw[2]+" "+str(px_noise)+" "+str(py_noise)+" "+str(pa_noise)+" "+lw[6]+" "+lw[7]+" "+lw[8]+" "+lw[9]+" "+lw[10]+" "+lw[11]+"\n")
    # else if line is edge XY        
    elif lw[0] == "EDGE_SE2_XY":
        lx_noise = float(lw[3]) + numpy.random.normal(0, 1/float(lw[5]))
        ly_noise = float(lw[4]) + numpy.random.normal(0, 1/float(lw[7]))
        f2.write(lw[0]+" "+lw[1]+" "+lw[2]+" "+str(lx_noise)+" "+str(ly_noise)+" "+lw[5]+" "+lw[6]+" "+lw[7]+"\n")
    # other lines
    else:
        f2.write(line)
f1.close()
f2.close()
