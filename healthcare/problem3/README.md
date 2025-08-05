# Here will be explained the usage

for now all that matters is that the commands that work to run is:

```
java -jar PANDA.jar -parser hddl domain.hddl problem.hddl | awk '/SOLUTION SEQUENCE/{flag=1; next} flag'  > plan.plan
```

which will generate a clean `plan.plan` file containing the only the full plan.