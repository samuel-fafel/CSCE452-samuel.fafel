import os
import time

if __name__ == '__main__':
    os.chdir('../../../')
    print("*** Building... ")
    os.system('colcon build')
    print("...Done ***\n")

    print("*** Sourcing... ", end="", flush=True)
    if os.system('. ./install/setup.sh'):
        print("Unable to Source! ***")
        exit()
    print("Done ***\n")

    print("*** Verifying... ", end="", flush=True)
    if os.system("ros2 pkg list | grep project3"):
        print("Unable to Find Package! ***")
        exit()
    print("Done ***\n")
        
    print("*** Launching ***")
    os.chdir('src/project3/project3/')
    for i in range(1,10):
        command = f"ros2 launch project3 launch.py bag_in:=bags/example{i}/example{i}.db3 bag_out:=bags/outputs/example{i}_output"
        os.system(command)

