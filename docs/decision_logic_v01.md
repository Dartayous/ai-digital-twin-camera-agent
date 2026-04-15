# Decision Logic v01

## Input Signal
target_visible

## States
- patrol
- focus_target

## Transition Rule
- if target_visible == false -> patrol
- if target_visible == true -> focus_target

## Required Behavior Meaning
patrol = maintain baseline scan behavior
focus_target = rotate camera agent toward LampTarget

## Rule
Decision logic must remain explicit, finite-state, and easy to debug.