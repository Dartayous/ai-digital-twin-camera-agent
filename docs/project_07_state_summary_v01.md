# Project 07 State Summary v01

## Milestone Achieved
Working deterministic perception -> decision -> action prototype

## Environment / Runtime Separation
- authored environment remains separate from runtime systems
- environment is referenced through /World/EnvWrapper
- runtime owns AgentRoot, camera, perception logic, decision logic, and control

## Stable Runtime Paths
- /World/AgentRoot
- /World/AgentRoot/AgentCamera
- /World/EnvWrapper/Environment/InspectionZone
- /World/EnvWrapper/Environment/LampTarget

## Working Perception Result
- camera and target world transforms resolved correctly
- target visibility computed deterministically from view alignment
- effective camera forward axis identified empirically
- v01 visibility threshold validated

## Working Decision Result
- patrol
- focus_target

## Working Action Result
- AgentRoot rotates toward LampTarget when target_visible == true

## Important Lessons
- do not assume camera forward axis from convention
- verify transforms from actual world-space data
- authored environment edits must happen in the authored file, not the composed runtime stage
- action visibility should be checked from an external viewport, not through the agent camera