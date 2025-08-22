# Healthcare Planning
Projects for Automated Planning course of the master degree in Artificial Intelligence Systems (AIS) at UniTN

# Setup
## Problems 1, 2 and 4
These problems require the usage of some planners that can be easily retrieved with planutils.
For simplicity a dockerfile is provided, to utilize it simply build it and run it by doing:
```
sudo docker build -t myplanutils .
```
And then run the container by doing:
```
sudo docker run -v ./healthcare:/computer -it --privileged --rm myplanutils bash
```

Note: only run it with the intention of testing the problems as to avoid issues with the others.

# Problem 3
For this problem there is no setup needed as the planner is already provided in the form of a `.jar` file.

# Problem 5
Similarly to before, a dockerfile is provided and is locate in the problem5 folder, to build do:
```
cd healthcare/problem5
sudo docker build -t ros -f Dockerfile.ROS .
```

And to run follow the instructions included in the README.md of problem5.

# Usage
Each problem folder contains a usage section in which there are all the informations needed.
Only relevant thing is to remember to correctly move in the folder by doing:
```
cd healthcare/problem*
```

And editing the path by substituting the `*` with the number of the problem.
Then, particularly for problem 1, 2 and 4, once the docker is run remember to move in the folder:
```
cd /computer
```
before following the folder usage instructions.
