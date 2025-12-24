# Implementation Plan: RAG Ingestion Pipeline

**Feature Branch**: `4-rag-ingestion-pipeline`  
**Feature Spec**: [spec.md](./spec.md)
**Created**: 2025-12-22

## 1. Technical Context

### Technology Stack
- **Language**: Python 3.9+
- **Package Manager**: `uv`
- **Crawling**: `requests` for HTTP requests, `BeautifulSoup4` for HTML parsing.
- **Embeddings**: `cohere` SDK.
- **Vector Database**: `qdrant-client` SDK.
- **Configuration**: `python-dotenv` for managing environment variables.
- **SiteMap URL**: https://physical-ai-humanoid-robotics-textb-eta.vercel.app/sitemap.xml

### Key Components
- **`main.py`**: A single script orchestrating the end-to-end ingestion pipeline.
  - **`get_all_urls`**: Crawls the target site recursively to find all content pages.
  - **`extract_text_from_url`**: Parses HTML to extract meaningful text.
  - **`chunk_text`**: Splits text into smaller, overlapping chunks.
  - **`embed`**: Generates vector embeddings using the Cohere API.
  - **`create_collection`**: Initializes the `rag_embedding` collection in Qdrant.
  - **`save_chunk_to_qdrant`**: Upserts chunk data and embeddings into Qdrant.

### External Dependencies
- **Cohere API**: Requires a valid API key for generating embeddings.
- **Qdrant Cloud/Instance**: Requires a URL and API key for storing and managing vectors.
- **Target Docusaurus Site**: The pipeline is designed to crawl a publicly accessible website.

## 2. Constitution Check

- **[✅] Accuracy**: All ingested content comes directly from the source URL.
- **[✅] Clarity**: The code will be structured with clear functions and comments for readability by senior CS/AI students.
- **[✅] Reproducibility**: The ingestion process is deterministic and can be reproduced given the same source content.
- **[✅] Zero Hallucination**: The pipeline only processes existing text; it does not generate new content.
- **[✅] Tested Code**: Unit and integration tests will be added in the implementation phase to verify each component.
- **[✅] Grounded RAG**: This pipeline is the first step to ensuring the RAG chatbot is grounded in the book's content.

## 3. Phase 0: Outline & Research

The primary research task was to determine a crawling strategy for the target SPA site. The findings are consolidated in the research document.

**Artifacts**:
- [research.md](./research.md)

## 4. Phase 1: Design & Contracts

The design phase defines the data structures and startup procedures for the pipeline.

**Artifacts**:
- [data-model.md](./data-model.md)
- [quickstart.md](./quickstart.md)

No formal API contracts are required as the initial implementation is a single-script pipeline. Function signatures in `main.py` serve as internal contracts.

## 5. Next Steps

With the plan, research, and design artifacts in place, the next step is to implement the functions in `backend/main.py`.
This will be handled in the `/sp.implement` or `/sp.tasks` phase.