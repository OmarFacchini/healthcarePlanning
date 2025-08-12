# Problem1

This is the basic situation in which we have the problem described in the main readme of the repository as well as in the `main.pdf` file.
Here the task is to have carriers move boxes containing one item around, effectively moving one item at a time with the purpose of delivering the correct item to the correct unit that requires it.

Same concept for patients.

## Usage
To run the problem use the command:

```
downward --alias lama-first domain.pddl problem.pddl
```

Which will generate a `sas_plan` file containing the full plan and its cost.