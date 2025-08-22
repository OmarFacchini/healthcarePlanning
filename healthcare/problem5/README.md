# Problem 5

This is a basic re-implementation of problem 4 to simulate actions throught ROS2 and Plansys2.

# Setup
it's important to make sure the image was setup correctly as showed in the main README.md of the repository.

# Usage
To run the problem two terminals are required, one for the launch and setup of the domain and one for the interactive problem and simulation.

## Terminal 1
In this terminal there will be the launch, which sets up the domain, to begin, start the docker container by doing:
```
sudo docker run -it --rm --network=host --name ubuntu_bash --env="DISPLAY" -v /tmp/.X11-unix/:/tmp/.X11-unix/ --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -v $HOME/Desktop/healthcarePlanning/healthcare/problem5:/root/plansys2_ws/src/problem5 -t ros bash
```

Make sure to properly set the path of the problem, in this case the project is assumed to be in the Desktop.

Then source the environment to be visible to the workspace and build dependencies by doing:
```
cd plansys2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Then launch by doing:
```
ros2 launch problem5 plansys2_healthcare_launcher.py
```
This will remain running and might be used for debug in case of issues.

## Terminal 2
Connect to the same container as terminal 1 by doing:
```
sudo docker exec -it ubuntu_bash bash
```

And similarly to before, source the environment by doing:
```
source /opt/ros/humble/setup.bash 
source plansys2_ws/install/setup.bash 
```

Then run the plansys2 terminal by doing:
```
ros2 run plansys2_terminal plansys2_terminal
```

Once this is done, set the problem by either sourcing it through the provided file `commands` by doing:
```
source /root/plansys2_ws/src/problem5/launch/commands
```
Or by writing it by hand exactly as in the file.

Then use Plansys2 to get a plan and run it by doing:
```
get plan
run
```

build the docker image:
cd healthcare/problem5
sudo docker build -t ros -f Dockerfile.ROS .


on terminal 2:
sudo docker exec -it ubuntu_bash bash

