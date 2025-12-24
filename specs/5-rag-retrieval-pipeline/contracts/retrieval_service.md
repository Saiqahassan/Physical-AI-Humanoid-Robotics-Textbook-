# API Contract: Retrieval Service Module

**Feature Branch**: `5-rag-retrieval-pipeline`
**Plan**: [plan.md](./plan.md)
**Created**: 2025-12-25
**Status**: Completed

This document defines the public-facing contract for the retrieval module (`backend/retrieval.py`).

## Function: `retrieve_chunks`

This is the main function that will be called by other backend services.

### Signature

```python
from typing import List, Optional, Dict

# Pydantic models defined in the data model
from .data_models import RetrievedChunk

def retrieve_chunks(
    query: str,
    filters: Optional[Dict[str, str]] = None,
    top_k: int = 5
) -> List[RetrievedChunk]:
    """
    Retrieves the most relevant document chunks from Qdrant based on a query.

    Args:
        query (str): The user's search query.
        filters (Optional[Dict[str, str]]): A dictionary of metadata key-value
            pairs to filter the search results. Supported keys are
            'source_url', 'chapter', and 'section'.
        top_k (int): The number of chunks to retrieve. Defaults to 5.

    Returns:
        List[RetrievedChunk]: A list of Pydantic objects, each representing a
            retrieved chunk with its text, source URL, and similarity score.
            Returns an empty list if no results are found.

    Raises:
        cohere.CohereAPIError: If there is an issue with the Cohere API.
        qdrant_client.http.exceptions.UnexpectedResponse: If Qdrant returns an
            unexpected response (e.g., validation error).
        ConnectionRefusedError: If the Qdrant service is not reachable.
    """
    # ... implementation ...
    pass
```

### Request & Response

-   **Request**: The arguments to the `retrieve_chunks` function, as defined in the signature.
-   **Response (Success)**: A `List[RetrievedChunk]`. The list will be empty if no results are found.
-   **Response (Error)**: The function will raise exceptions for error conditions, as documented in the docstring. The calling service is responsible for handling these exceptions.
