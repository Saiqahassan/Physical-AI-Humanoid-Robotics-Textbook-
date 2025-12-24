# Research: Retrieval Pipeline Best Practices

**Feature Branch**: `5-rag-retrieval-pipeline`
**Plan**: [plan.md](./plan.md)
**Created**: 2025-12-25
**Status**: Completed

## 1. Shared Configuration in Python

### Decision

We will adopt a centralized configuration module using Pydantic's `BaseSettings`.

-   A `config.py` file will be created at `backend/core/config.py`.
-   This module will define a `Settings` class inheriting from `pydantic_settings.BaseSettings`.
-   It will be configured to automatically load variables from a `.env` file located in the `backend/` directory.
-   A cached function `get_settings()` will provide a singleton instance of the `Settings` object to the rest of the application.

### Rationale

This approach, leveraging Pydantic, provides several key advantages:

-   **Type Safety**: Ensures that environment variables are loaded with the correct data types, preventing common runtime errors.
-   **Validation**: Pydantic automatically validates the configuration on startup, catching issues like missing required variables early.
-   **Centralization**: Consolidates all configuration into a single, predictable location, making it easy to manage.
-   **Compatibility**: It integrates seamlessly with FastAPI (which will likely be used for the chatbot application) for dependency injection.

### Alternatives Considered

-   **Global `os.getenv()` calls**: Directly calling `os.getenv()` wherever a variable is needed. This was rejected because it's not centralized, offers no type safety, and makes configuration hard to track.
-   **Simple Global Variables**: Defining variables in a `config.py` file. This is better than `os.getenv()` but lacks the validation and type safety of the Pydantic approach.

---

## 2. Exception Handling for `qdrant-client` and `cohere`

### Decision

We will implement specific `try...except` blocks to catch the primary exception classes from each library.

-   **For `qdrant-client`**:
    -   Catch `qdrant_client.http.exceptions.UnexpectedResponse` for API validation errors (e.g., bad filters, incorrect vector dimensions).
    -   Catch `qdrant_client.http.exceptions.ApiException` as a general fallback for other Qdrant-specific API issues.
    -   Catch standard `ConnectionRefusedError` for when the Qdrant instance is not reachable.

-   **For `cohere`**:
    -   Catch `cohere.CohereAPIError` for all API-related errors, including invalid API keys, rate limiting (`429`), and bad requests (`400`).

### Rationale

Catching specific exceptions allows the application to handle different failure modes gracefully. For instance, a connection error might be retried, while an invalid API key error should cause a hard failure with a clear error message. This makes the retrieval service more robust and easier to debug.

### Alternatives Considered

-   **Catching generic `Exception`**: A single `except Exception:` block could be used, but it hides the specific cause of the error, making it difficult to implement targeted error handling or provide meaningful logs.
