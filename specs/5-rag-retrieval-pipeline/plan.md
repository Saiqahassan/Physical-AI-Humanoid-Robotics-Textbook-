# Implementation Plan: Retrieval Pipeline for RAG Chatbot

**Feature Branch**: `5-rag-retrieval-pipeline`
**Feature Spec**: [spec.md](./spec.md)
**Created**: 2025-12-25
**Status**: Draft

## 1. Technical Context

### 1.1. Technology Stack

-   **Language**: Python
-   **Vector Database**: Qdrant
-   **Embedding Provider**: Cohere

### 1.2. Key Components & Approach

The plan is to create a single, modular Python script at `backend/retrieval.py`. This script will encapsulate all the logic for the retrieval process.

1.  **Configuration Management**:
    -   A `.env` file within the `backend/` directory will store secrets (Cohere API Key, Qdrant API Key, Qdrant URL).
    -   The `python-dotenv` library will be used to load these environment variables.

2.  **Query Embedding**:
    -   A dedicated function will take a raw string query.
    -   It will use the `cohere` client to generate an embedding using the `embed-english-v3.0` model, consistent with the ingestion pipeline.

3.  **Vector Search**:
    -   A core function will connect to the Qdrant client.
    -   It will perform a `search` operation using the generated query vector.
    -   It will support filtering based on metadata fields (`source_url`, `chapter`, `section`) passed as a dictionary.
    -   The search will use cosine similarity and retrieve the top-K results (default 5, max 25).

4.  **Result Processing & Data Structures**:
    -   A Pydantic `BaseModel` will define the structure of the `RetrievedChunk` (text, source_url, score) for clear data contracts.
    -   The main retrieval function will return a list of these Pydantic models.

### 1.3. Integration Points

-   This `retrieval.py` module is designed to be imported and used by other backend services, specifically the main RAG chatbot application logic which is not yet built. The chatbot will call the primary retrieval function in this module to get context for a user's query.

### 1.4. External Dependencies

-   `cohere`: For generating query embeddings.
-   `qdrant-client`: For interacting with the Qdrant vector database.
-   `python-dotenv`: For managing environment variables.
-   `pydantic`: For data validation and structured output.

### 1.5. Risks & Unknowns

-   **Module Location**: The prompt was slightly ambiguous about whether to create `retrieve.py` in the root or `backend/retrieval/`. The decision is to place it at `backend/retrieval.py` to keep backend logic consolidated. **[RESOLVED]**
-   **Configuration Loading**: Need to confirm the best practice for loading `.env` in a modular way that the future chatbot application can also use. **[NEEDS CLARIFICATION]**
-   **Error Handling**: Specific Qdrant/Cohere exceptions need to be identified to provide robust error handling (e.g., for timeouts, invalid keys). **[NEEDS CLARIFICATION]**

## 2. Constitution Check

-   **Accuracy**: The plan relies on the accuracy of the ingested data. The retrieval logic itself is a standard vector search.
-   **Clarity**: The code will be structured with clear function responsibilities and Pydantic models for clarity.
-   **Reproducibility**: The retrieval process is deterministic for a given query and dataset version.
-   **Zero Hallucination**: N/A for retrieval, but the retrieved content is grounded in the source text.
-   **Tested Code**: The plan includes adding basic validation checks.
-   **Grounded RAG**: This is the core component for grounding the RAG system.

**Result**: The plan is compliant with the constitution.

## 3. Phase 0: Research

Research has been completed and documented in [research.md](./research.md). All initial unknowns have been resolved.

## 4. Phase 1: Design & Contracts

The design and contracts for the retrieval service are complete.

-   **Data Model**: [data-model.md](./data-model.md)
-   **API Contract**: [contracts/retrieval_service.md](./contracts/retrieval_service.md)
-   **Quickstart Guide**: [quickstart.md](./quickstart.md)
