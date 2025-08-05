# Here will be explained the usage

for now all that matters is that the commands that work to run is:

```
java -jar PANDA.jar -parser hddl domain.hddl problem.hddl | awk '/SOLUTION SEQUENCE/{flag=1; next} flag' > plan.plan
```

which will generate a clean `plan.plan` file containing the only the full plan.

NOTE: this works only for patients for now, will require the usage of Panda-PI to include the usage of integers which are needed for the limitations of weight that each container can maintain.