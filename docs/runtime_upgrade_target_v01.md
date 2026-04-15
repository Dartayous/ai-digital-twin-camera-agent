# Runtime Upgrade Target v01

## Current State
Working file-driven perception -> decision -> action loop is validated.

## Next Upgrade Goal
Move from file-driven polling to true Isaac Sim runtime execution.

## Required Improvement
The agent loop must run against the live stage in Isaac Sim, not by reopening and saving the USD file repeatedly.

## Why
This will move the project from prototype behavior to simulation-grade runtime behavior.

## Rule
Do not add new perception complexity until the runtime execution model is upgraded.