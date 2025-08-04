# Here will be explained the usage

This works, but is very VERY bad and takes hours for this simple task, therefore while it's an option I highly suggest ***avoiding*** it

```
enhsp -o domain.pddl -f problem.pddl > filename
```

Currently the best planner and the related command to use is:

```
metric-ff domain.pddl problem.pddl  > filename
```

which will generate two files:

1. `filename` which contains the full output of the planner with extra informations (eg. time spent)
2. `problem.pddl.plan` which contains only the plan