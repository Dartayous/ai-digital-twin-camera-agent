# Action Logic v01

## Controlled Prim
/World/AgentRoot

## Input State
focus_target

## Action Definition
When state == focus_target, rotate AgentRoot so the agent faces:
/World/EnvWrapper/Environment/LampTarget

## Patrol Definition
When state == patrol, AgentRoot maintains its baseline scan pose or scripted scan motion.

## Rule
Action must be visibly observable in the viewport.
The reviewer must clearly see the orientation change.