# Quickstart: RAG Ingestion Pipeline

**Date**: 2025-12-22

This guide provides instructions on how to set up and run the RAG ingestion pipeline.

## 1. Prerequisites

- Python 3.9+
- `uv` installed (`pip install uv`)
- Access to a Qdrant instance
- A Cohere API key

## 2. Setup

### a. Clone the Repository
```bash
git clone <repository_url>
cd <repository_url>
```

### b. Create a Virtual Environment
Navigate to the `backend` directory and create a virtual environment.
```bash
cd backend
uv venv
```

### c. Install Dependencies
Activate the environment and install the required packages. `uv` will automatically detect and install dependencies from `pyproject.toml` when we add them. For now, we will add them manually.
```bash
source .venv/bin/activate  # On Windows use `.venv\Scripts\activate`
uv pip install requests beautifulsoup4 cohere qdrant-client python-dotenv
```

### d. Configure Environment Variables
Create a `.env` file in the `backend` directory and add your credentials:
```env
COHERE_API_KEY="your_cohere_api_key"
QDRANT_API_KEY="your_qdrant_api_key"
QDRANT_URL="your_qdrant_instance_url"
```

## 3. Running the Pipeline

Once the setup is complete, you can run the ingestion pipeline by executing the `main.py` script:
```bash
python main.py
```

The script will crawl the target site, process the content, and populate your Qdrant collection. You can monitor the progress via the console output.
