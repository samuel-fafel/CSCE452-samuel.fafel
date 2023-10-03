import os

def main(args=None):
	os.system("ros2 param set /turtlesim background_r 80")
	os.system("ros2 param set /turtlesim background_g 0")
	os.system("ros2 param set /turtlesim background_b 0")

if __name__ == "__main__":
    main()
