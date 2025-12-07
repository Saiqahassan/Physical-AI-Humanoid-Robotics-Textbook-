---
id: 1
title: Create Project Constitution
stage: constitution
date: 2025-12-07
surface: agent
model: gemini
feature: none
branch: master
user: Saiqahassan
command: /sp.constitution
labels: [constitution, project-setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
- .specify/templates/plan-template.md
- .specify/templates/tasks-template.md
- .gemini/commands/sp.constitution.toml
- .gemini/commands/sp.analyze.toml
- .gemini/commands/sp.checklist.toml
- .gemini/commands/sp.clarify.toml
- .gemini/commands/sp.git.commit_pr.toml
- .gemini/commands/sp.implement.toml
- .gemini/commands/sp.phr.toml
- .gemini/commands/sp.plan.toml
- .gemini/commands/sp.specify.toml
- .gemini/commands/sp.tasks.toml
tests:
- none
---

## Prompt

---
description: Create or update the project constitution from interactive or provided principle inputs, ensuring all dependent templates stay in sync
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


/sp.constitution Project: Physical AI & Humanoid Robotics  Deliverables:  1) Docusaurus book deployed to GitHub Pages  2) Integrated RAG chatbot (OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant)Purpose:  Create a clear, technically accurate book teaching students how AI systems operate in the physical world using ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action systems.  All content must be reproducible, rigorous, and grounded in authoritative sources.Core Principles:  - Accuracy from primary/official robotics sources  - Clarity for senior CS/AI students  - Reproducible simulations and code  - Zero hallucination  - All claims traceable and cited (APA)Standards:  - Tested code (ROS 2, Gazebo, Unity, Isaac)  - Consistent pedagogy: Intro → Theory → Code → Example  - Diagrams for complex systems  - RAG chatbot answers only from book content or user-selected text  - No fictional APIs or robotics claimsDomain Scope:  Physical AI, humanoid robotics, ROS 2, URDF, Gazebo physics, Unity rendering, Isaac Sim/ROS, SLAM, Nav2, VLA, Whisper voice commands, cognitive planning, RAG systems.Structure (High-level):  Module 1 — ROS 2 (robotic nervous system)  Module 2 — Gazebo & Unity (digital twin)  Module 3 — NVIDIA Isaac (AI-robot brain)  Module 4 — VLA (language → action)  Capstone — Autonomous humanoidConstraints:  - Accurate, runnable examples  - Zero plagiarism  - Chatbot grounded 100% in text  - Ubuntu 22.04, ROS 2 Humble/Iron, Gazebo, Isaac 4.x  - No unsafe robot-operation instructionsSuccess Criteria:  - Book builds cleanly in Docusaurus  - All tutorials reproducible  - Code runs without modification  - RAG retrieval accuracy ≥ 90%  - No hallucinationsVoice & Tone:  Technical, precise, instructor-level, robotics-focused.Rules:  - All future `/sp.*` outputs must comply with this constitution.  - If a request conflicts, respond: “Request violates constitution.”

## Response snapshot

I have created the project constitution and updated all dependent files.

## Outcome

- ✅ Impact: The project constitution was created and all dependent templates and commands were updated to be consistent with it.
- 🧪 Tests: none
- 📁 Files: 14 files updated.
- 🔁 Next prompts: none
- 🧠 Reflection: The initial setup of the project constitution is a critical step. It was important to not only create the constitution but also to ensure that all templates and command files were consistent with it. This will help to ensure that all future development work is aligned with the project's core principles.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): none
