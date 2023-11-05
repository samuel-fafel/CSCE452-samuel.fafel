import os

if __name__ == "__main__":
    for i in range(1,10):
        command = f"ros2 bag play bags/example{i}/example{i}.db3"
        os.system(command)

        