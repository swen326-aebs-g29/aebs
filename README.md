# AEBS - Automatic Emergency Braking System
Testing

SWEN326 Group Project | Group 29 - Safe Mode | Victoria University of Wellington 2026

## Overview

A safety-critical Automatic Emergency Braking System (AEBS) developed following IEC-61508 and ISO-2626 practices. The system detects obstacles via simulated sensors and applies emergency braking when required.

The project consists of two components:
- **Core system** (`/core`) - the safety-critical AEBS logic that would run in a vehicle
- **Simulator** (`/simulator`) - a test environment that feeds realistic and fault-injected sensor data to the core system

## Team

| Role | Person |
|---|---|
| Documentation Lead | Khai |
| Core AEBS Developer | Gulshan |
| Simulator Developer | Wa |
| Traceability & Testing | Molly |

## Repository Structure
```
aebs/
├── core/               # Safety-critical AEBS system
│   └── src/
│       ├── main/java/
│       └── test/java/
├── simulator/          # Sensor simulation and fault injection
│   └── src/
│       ├── main/java/
│       └── scenario/
├── docs/               # Project documentation
│   ├── project-plan/
│   ├── hara-fta/
│   ├── architecture/
│   └── meeting-minutes/
└── traceability/       # Requirements traceability matrix
```

## Development

- **Language:** Java 25 (Eclipse Temurin)
- **Coding standard:** Power of Ten
- **Branch protection:** All changes via pull request, minimum 1 review required

### Commit message format
All commits must include a requirement ID:
```
feat(REQ-012): implement braking decision logic
fix(REQ-023): correct retry count in braking control module
```

## Submissions

| Deliverable | Due |
|---|---|
| Individual report (`StudentID_report.pdf`) | 11:59pm Friday 15 May |
| Group submission (`29_ProjectPlan.pdf`, `29_RequirementsTrace.xlsx`) | Midday Monday 18 May |
| In-person interview | Week 11, Cotton 241 (19-22 May) |