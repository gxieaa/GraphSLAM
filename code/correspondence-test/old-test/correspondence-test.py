# imports
import shlex
import string

landmarks = []
filename = "guess_out/guess_out_i_10_s_300_l_300_p_500_a_5000_ps_1000.g2o"
f1 = open(filename, 'r')
f2 = open(filename, 'r')
f3 = open(filename, 'r')
marks = createMarks()

for line1 in f1:
    i = i+1
    if string.find(line1, "VERTEX_XY") > -1:
        vertexXYWords = shlex.split(line1)
        #print("vertexXY:")
        #print(vertexXYWords)
        for line2 in f2:
            if string.find(line2, "EDGE_SE2_XY") > -1:
                edgeWords = shlex.split(line2)
                #print("edge:")
                #print(edgeWords)
                if edgeWords[2] == vertexXYWords[1]:
                    for line3 in f3:
                        if string.find(line3, "VERTEX_SE2") > -1:
                            #print("edge:")
                            #print(edgeWords)
                            vertexSE2Words = shlex.split(line3)
                            if vertexSE2Words[1] == edgeWords[1]:
                                lx, ly = getLPos()
                                landmarks.append(lx, ly, marks[i%len(marks)])
                                print(vertexXYWords[1]+" - "+vertexSE2Words[1])
                    f3.seek(0)
        f2.seek(0)
        
def createMarks():
    colors = ['b','g','r','c','m','y','k']
    markers = ['.','o','v','^','<','>','1','2','3','4','8','s','p','*','h','H','+','x','D']
    marks = []
    for m in markers:
        for c in colors:
            mark.append(c+m)
