# Integration Contract v01

## Environment-owned by authored scene
- room geometry
- desk geometry
- lamp geometry
- visual materials
- lighting design

## Runtime-owned by simulation layer
- AgentRoot
- agent-mounted sensors
- decision state
- control script
- debug visualization if needed

## Required authored targets for integration
- one desk inspection zone marker
- one lamp target marker

## Required rule
Runtime systems must attach to stable prim paths and must not depend on manual viewport operations.