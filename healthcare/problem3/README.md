# Problem3

This is an extension of Problem2 in which we use Hierarchical Domain Definition Language (HDDL) to define the problem as an Hierarchical Task Network (HTN).

## There are two types of domain and problem in this task:
1. `recursive`: where the tasks are recursive and more general, allowing the planner to infer by itself the amount of boxes to load/unload according to the limit.
2. `simple`: where the tasks are fixed for this specific problem with a fixed limit and not easily scalable.

## Pros and Cons
### Recursive
- Easier to scale to multiple boxes/container limits
- Much slower to run

### Simple
- Harder to scale, need to create extra specific functions per new limit
- Much faster than its counterpart

## Usage
### Recursive 
Run the following command to have generate a clean file containing only the steps of the resulting plan.
```
java -jar PANDA.jar -parser hddl domain_rec.hddl problem_rec.hddl | awk '/SOLUTION SEQUENCE/{flag=1; next} flag' > plan_rec.plan
```

To get the plan as well as extra informations and keep them in the file, simply run:
```
java -jar PANDA.jar -parser hddl domain_rec.hddl problem_rec.hddl > plan_rec.plan
```

### Simple
For the simple domain there are two problems:
1. Really simple one in which the items are delivered one by one, not taking advantage of the container capacity
2. Slightly more complex in which more than one item can be delivered at a time

To run the simple (first) one use the command:
```
java -jar PANDA.jar -parser hddl domain_simple.hddl problem_simple.hddl | awk '/SOLUTION SEQUENCE/{flag=1; next} flag' > plan_simple.plan
```

To run the complex (second) one use the command:
```
java -jar PANDA.jar -parser hddl domain_rec.hddl problem_rec.hddl | awk '/SOLUTION SEQUENCE/{flag=1; next} flag' > plan_rec.plan
```


NOTE: change the `filename.plan` name accordingly to however you want the file to be saved as.