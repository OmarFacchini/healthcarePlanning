# Problem2

This is an extension of problem1 in which we add a container to the carriers allowing them to move more than 1 box at a time up to a defined limit.

## Usage
To run the problem use the command:
```
metric-ff domain.pddl problem.pddl > actions
```

which will generate two files:

1. `actions` which contains the full output of the planner with extra informations (eg. time spent) as well as the plan
2. `problem.pddl.plan` which contains only the clean plan

NOTE: change the `actions` name accordingly to however you want the file to be saved as.