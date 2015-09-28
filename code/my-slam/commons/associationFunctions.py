# imports
import shlex
import string

def anonymizeLandmarks(inPath, outPath):
    fIn = open(inPath, 'r')
    fOut = open(outPath, 'w')
    count = 10000
    
    for line in fIn:
        # split string
        newline = line
        lineWords = shlex.split(newline)
        # anonimize
        if string.find(lineWords[0], "EDGE_SE2_XY") != -1:
            lineWords[2] = str(count)
            count = count + 1
            newline = ' '.join(lineWords) + '\n'
        fOut.write(newline)
    fIn.close()
    fOut.close()
