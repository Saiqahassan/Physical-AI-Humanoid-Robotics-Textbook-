# Data Model: Retrieval Pipeline

**Feature Branch**: `5-rag-retrieval-pipeline`
**Plan**: [plan.md](./plan.md)
**Created**: 2025-12-25
**Status**: Completed

This document defines the key data structures for the RAG retrieval pipeline. These will be implemented using Pydantic for data validation and clear contracts.

## 1. `RetrievalRequest` (Input)

This model represents the input to the retrieval service.

-   **Fields**:
    -   `query` (str): The raw text query from the user. (Required)
    -   `filters` (Optional[Dict[str, str]]): A dictionary of metadata filters to apply to the search. Keys can include `source_url`, `chapter`, and `section`. (Optional)
    -   `top_k` (int): The number of results to return. Defaults to `5`.

-   **Validation Rules**:
    -   `query` must be a non-empty string.
    -   `top_k` must be an integer between 1 and 25 (the maximum limit).

## 2. `RetrievedChunk` (Output)

This model represents a single retrieved document chunk returned by the service. The service will return a list of these objects.

-   **Fields**:
    -   `text` (str): The actual text content of the document chunk.
    -   `source_url` (str): The original source URL from which the chunk was extracted.
    -   `score` (float): The cosine similarity score from Qdrant, indicating the relevance of the chunk to the query.

-   **Validation Rules**:
    -   All fields are required.
    -   `score` must be a float, typically between 0.0 and 1.0 for cosine similarity.
