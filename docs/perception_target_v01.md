# Perception Target v01

## Target Prim
/World/EnvWrapper/Environment/LampTarget

## Detection Definition (v01)
The agent does NOT use ML.

Detection = target is inside the camera’s forward view cone.

## Required Data
- camera world transform
- target world transform

## Output Signal
target_visible = true / false

## Rule
Perception must be deterministic and debuggable.
No randomness.
No black-box behavior.