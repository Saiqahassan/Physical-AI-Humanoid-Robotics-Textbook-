# Quickstart: RAG Retrieval Service

**Feature Branch**: `5-rag-retrieval-pipeline`
**Plan**: [plan.md](./plan.md)
**Created**: 2025-12-25
**Status**: Completed

This guide provides instructions on how to use the RAG retrieval service module.

## 1. Prerequisites

-   Python 3.9+
-   An active Qdrant instance populated with data from the ingestion pipeline.
-   Valid API keys for Cohere and Qdrant.

## 2. Setup

1.  **Install Dependencies**:
    ```bash
    pip install cohere qdrant-client python-dotenv pydantic
    ```

2.  **Configure Environment**:
    -   Create a `.env` file in the `backend/` directory.
    -   Add your API keys and Qdrant URL to this file:
        ```env
        COHERE_API_KEY="your_cohere_key"
        QDRANT_API_KEY="your_qdrant_key"
        QDRANT_URL="http://localhost:6333"
        ```

## 3. Usage Example

The following script demonstrates how to import and use the `retrieve_chunks` function.

```python
import os
from backend.retrieval import retrieve_chunks

def main():
    """
    Example of how to call the retrieval service.
    """
    query = "What is the role of a state estimator in robotics?"

    print(f"Searching for: '{query}'")

    # Basic retrieval
    results = retrieve_chunks(query=query, top_k=3)

    print(f"Found {len(results)} results:")
    for i, chunk in enumerate(results):
        print(f"  {i+1}. Score: {chunk.score:.4f}, Source: {chunk.source_url}")
        # print(f"      Text: {chunk.text[:100]}...") # Uncomment to see text

    # Retrieval with metadata filter
    print("\nSearching with a filter...")
    filters = {"source_url": "https://physical-ai-humanoid-robotics-textb-eta.vercel.app/docs/intro"}
    filtered_results = retrieve_chunks(query=query, filters=filters, top_k=2)

    print(f"Found {len(filtered_results)} filtered results:")
    for i, chunk in enumerate(filtered_results):
        print(f"  {i+1}. Score: {chunk.score:.4f}, Source: {chunk.source_url}")


if __name__ == "__main__":
    # Assumes your .env file is in the parent directory relative to this script
    # This is just for demonstration; the module itself will use a config loader.
    from dotenv import load_dotenv
    load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), '..', '.env'))
    main()
```
