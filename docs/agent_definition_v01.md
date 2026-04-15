# Agent Definition v01

## Agent Type
Camera Agent

## Runtime Form
A lightweight flying inspection agent represented as a controllable Xform with an attached camera sensor.

## Core Behavior
- patrols the room
- scans the desk zone
- detects the lamp target area
- rotates toward the target
- transitions into inspection focus behavior

## Why this agent was chosen
This project avoids dependence on fragile prebuilt robot stacks and instead uses a controlled agent architecture with a simpler motion + sensing loop.

## Required Runtime Components
- agent root prim
- camera prim attached to agent
- scripted motion controller
- target zone marker
- perception signal
- decision state
- visible action change

## v01 Success Condition
The agent moves through the environment, identifies the lamp zone, and visibly reorients toward it.