# imports
import subprocess

def main():
    # variables
    binPath = "../../g2o-master/bin/"
    resPath = "res.g2o"
    
    subprocess.call([binPath+"./g2o_simulator2d", 
    "-hasOdom", "-hasPointSensor",
    "-nlandmarks", "50",
     "-simSteps", "10",
    resPath])

if __name__ == '__main__':
    main()
