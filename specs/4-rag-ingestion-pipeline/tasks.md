# Actionable Tasks: RAG Ingestion Pipeline

**Feature**: Content Ingestion, Embedding, and Vector Storage Pipeline for RAG Chatbot
**Branch**: `4-rag-ingestion-pipeline`

This document breaks down the implementation of the RAG Ingestion Pipeline into actionable, dependency-ordered tasks.

## Phase 1: Setup

These tasks initialize the project and environment.

- [X] T001 [P] Create a `.env` file in `backend/` with placeholders for `COHERE_API_KEY`, `QDRANT_API_KEY`, and `QDRANT_URL`.
- [X] T002 [P] Install all required dependencies (`requests`, `beautifulsoup4`, `cohere`, `qdrant-client`, `python-dotenv`) using `uv pip install`.

## Phase 2: Foundational (P1 Story Prerequisites)

These tasks create the foundational components required for the main ingestion workflow.

- [X] T003 Implement the Qdrant client initialization in `backend/main.py`.
- [X] T004 Implement the `create_collection` function to set up the `rag_embedding` collection in `backend/main.py`.

## Phase 3: User Story 1 - Core Ingestion Pipeline (P1)

**Goal**: Configure and run the full ingestion pipeline.
**Independent Test**: The pipeline runs end-to-end, ingesting content from a sample URL and storing it in Qdrant.

- [X] T005 [P] [US1] Implement the `get_all_urls` function in `backend/main.py` using the recursive crawling strategy from `research.md`.
- [X] T006 [P] [US1] Implement the `extract_text_from_url` function in `backend/main.py` to parse HTML and extract text.
- [X] T007 [P] [US1] Implement the `chunk_text` function in `backend/main.py`.
- [X] T008 [P] [US1] Implement the `embed` function in `backend/main.py` to call the Cohere API.
- [X] T009 [US1] Implement the `save_chunk_to_qdrant` function in `backend/main.py` to upsert data.
- [X] T010 [US1] Implement the main execution logic in the `main` function to orchestrate the pipeline in `backend/main.py`.

## Phase 4: User Story 2 - Incremental Updates (P2)

**Goal**: Handle content updates efficiently without a full re-ingestion.
**Independent Test**: Running the pipeline again on a modified source document updates only the changed content in Qdrant.

- [X] T011 [P] [US2] Add a `checksum` field to the data model in `backend/main.py`.
- [X] T012 [US2] Modify `save_chunk_to_qdrant` in `backend/main.py` to store the checksum with each chunk.
- [X] T013 [US2] Update the main pipeline loop in `backend/main.py` to:
    - a. Fetch existing chunks for a URL from Qdrant.
    - b. Compare checksums to identify new, updated, or deleted content.
    - c. Only process changes.

## Phase 5: Polish & Cross-Cutting Concerns

These tasks improve the robustness and usability of the pipeline.

- [X] T014 [P] Add detailed logging for all major steps of the pipeline in `backend/main.py`.
- [X] T015 [P] Implement robust error handling and retry logic for network requests and API calls in `backend/main.py`.

## Dependencies & Execution Strategy

- **User Story 1 (T003-T010)** is the **MVP** and can be implemented and delivered first.
- **User Story 2 (T011-T013)** depends on the completion of User Story 1.
- **Polish tasks (T014-T015)** can be worked on in parallel with other tasks or addressed after the core functionality is complete.

### Parallel Execution Examples

- Within **US1**, tasks `T005`, `T006`, `T007`, `T008` can be developed in parallel as they are independent function implementations. `T009` and `T010` depend on them.
- **Polish tasks** (`T014`, `T015`) are highly parallelizable and can be implemented at any time after the initial function stubs are in place.
