Backend quick start — Qdrant & Cohere
===================================

Quick steps to run the ingestion pipeline locally.

1) Create a `.env` in `backend/` based on `.env.example` and fill your API keys.

2) Run a local Qdrant:

```bash
docker-compose up -d
```

This brings up Qdrant on `http://localhost:6333` by default.

3) Run the ingestion script:

```bash
python main.py
```

Troubleshooting
- If you see connection timeouts to Qdrant, check:
  - `backend/.env` values: `QDRANT_URL` should be reachable from your machine.
  - Firewall or corporate proxy blocking outbound traffic.
  - If using a cloud Qdrant, ensure the URL and API key are correct and the service is running.

Local API key note
- The compose file will pass `QDRANT_API_KEY` into the container as `QDRANT__SERVICE__API_KEY` if provided. For local testing you can leave `QDRANT_API_KEY` blank.
