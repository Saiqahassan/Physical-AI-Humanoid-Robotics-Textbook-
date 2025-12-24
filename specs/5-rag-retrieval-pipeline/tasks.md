# Tasks: Retrieval Pipeline for RAG Chatbot

**Feature Branch**: `5-rag-retrieval-pipeline`
**Total Tasks**: 10
**MVP Scope**: All tasks (T001-T010) are required for the MVP.

This document outlines the tasks required to implement the RAG retrieval pipeline.

## Implementation Strategy

The feature will be developed in a single, sequential pass, as each task builds directly on the previous one. The MVP consists of a fully functional and verifiable retrieval module.

---

## Phase 1: Setup
*Initial project setup and dependency configuration.*

- [x] T001 Create the directory structure `backend/core/` and `backend/retrieval/`
- [x] T002 Add `cohere`, `qdrant-client`, `python-dotenv`, and `pydantic` to the `[tool.uv.sources]` and `[project.dependencies]` sections of `backend/pyproject.toml`

---

## Phase 2: Foundational
*Core configuration and data models that underpin the feature.*

- [x] T003 Implement the shared Pydantic `Settings` module in `backend/core/config.py` as defined in `research.md`
- [x] T004 Create a `.env.example` file in the `backend/` directory with placeholders for `COHERE_API_KEY`, `QDRANT_API_KEY`, and `QDRANT_URL`
- [x] T005 [P] Define the Pydantic `RetrievalRequest` and `RetrievedChunk` models in `backend/retrieval/data_models.py`

---

## Phase 3: User Story 1 - Implement Core Retrieval Logic
*Implement the main functionality of retrieving document chunks.*

**Goal**: A backend service can call a function to get a ranked list of relevant text chunks for a given query.
**Independent Test**: A standalone script can call the `retrieve_chunks` function and print the results, verifying the output structure and relevance.

- [x] T006 [US1] Implement the main `retrieve_chunks` function in `backend/retrieval/main.py`, including query embedding with Cohere
- [x] T007 [US1] Integrate Qdrant vector search with metadata filtering into the `retrieve_chunks` function in `backend/retrieval/main.py`
- [x] T008 [US1] Implement robust exception handling for Cohere and Qdrant API errors in `backend/retrieval/main.py`

---

## Phase 4: Polish & Validation
*Finalizing the feature with documentation and a validation script.*

- [x] T009 [P] Create a validation script at `backend/validation/test_retrieval.py` that uses the `quickstart.md` example to run a live test
- [x] T010 Update the main `backend/README.md` to include instructions on how to use the new retrieval module and run the validation script

## Dependencies

-   **User Story 1** is dependent on the completion of **Phase 1 (Setup)** and **Phase 2 (Foundational)**.

## Parallel Execution

-   **T005** can be implemented in parallel with **T003** and **T004**.
-   **T009** can be started once the contract (function signature) in **T006** is defined, even before the full implementation is complete.
