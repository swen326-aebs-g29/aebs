# Contributing to AEBS

## Workflow

1. Pick up an issue from the project board and move it to In Progress
2. Create a branch from main using the naming convention below
3. Do your work and commit regularly using the commit format below
4. Push your branch and open a pull request using the PR template
5. Assign at least one reviewer
6. Once approved, merge into main and close the issue

## Branch Naming

Use this format: type/short-description

Examples:
- feat/sensor-interface-module
- fix/braking-retry-logic
- docs/update-architecture-diagram
- test/unit-tests-voting-logic

## Commit Message Format

Every commit must include a requirement ID:

  type(REQ-XXX): short description

Types: feat, fix, docs, test, chore, refactor

Examples:
- feat(REQ-012): implement braking decision logic
- fix(REQ-023): correct retry count in braking control module
- docs(REQ-001): update architecture diagram
- test(REQ-015): add unit tests for sensor interface

## Pull Requests

- Use the PR template — fill in every section
- Link the related issue using Closes #issue-number
- Make sure all acceptance criteria on the issue are met before requesting review
- At least 1 approval required before merging

## Coding Standard

All code in /core must follow the Power of Ten rules.
Simulator code in /simulator does not need to follow ISO 26262 but should still be clean and readable.

## Traceability

- Include a requirement ID in every commit message
- Notify Molly when a new module is merged so the RTM can be updated
- No merge without a requirement ID — this will be checked in the Week 11 interview

## Java Setup

- Language: Java 25
- Vendor: Eclipse Temurin
- Set up via File > Project Structure > SDKs > Download JDK in IntelliJ

## Need Help?

Post in the Discord #development channel.
