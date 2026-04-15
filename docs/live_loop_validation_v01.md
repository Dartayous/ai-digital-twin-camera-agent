# Live Loop Validation v01

## Verified Behavior
- continuous perception loop runs successfully
- LampTarget movement causes AgentRoot reorientation
- target visibility drives explicit state changes
- loop can be stopped cleanly with Ctrl+C

## Current Runtime Model
This is a file-driven prototype loop.
The script reopens the USD stage, evaluates perception, updates action, saves, and repeats.

## Important Constraint
This is valid for v01 validation, but it is not the final runtime architecture for Isaac Sim.

## v01 Success
Perception -> Decision -> Action is now live and observable.