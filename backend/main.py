import logging
import os
import hashlib
from urllib.parse import urljoin, urlparse

import requests
from bs4 import BeautifulSoup
from dotenv import load_dotenv
from tenacity import retry, stop_after_attempt, wait_fixed
import uuid
import sys
from tenacity import RetryError

import cohere
import qdrant_client
from qdrant_client.http.models import (
    VectorParams,
    Distance,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue,
    PointsSelector,
)

# --------------------------------------------------
# ENV & LOGGING
# --------------------------------------------------

load_dotenv()

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
)

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")

COLLECTION_NAME = "rag_embedding"
EMBEDDING_SIZE = 1024 # ✅ Cohere embed-english-v3.0

# --------------------------------------------------
# HTTP HELPERS
# --------------------------------------------------

@retry(stop=stop_after_attempt(3), wait=wait_fixed(2))
def _robust_requests_get(url: str):
    response = requests.get(url, timeout=10)
    response.raise_for_status()
    return response


# --------------------------------------------------
# CRAWLING
# --------------------------------------------------

def get_all_urls(base_url: str):
    logging.info(f"Starting crawl: {base_url}")
    base_domain = urlparse(base_url).netloc

    urls_to_visit = {base_url}
    visited_urls = set()

    while urls_to_visit:
        url = urls_to_visit.pop()
        if url in visited_urls:
            continue

        visited_urls.add(url)
        logging.info(f"Crawling: {url}")

        try:
            response = _robust_requests_get(url)
            soup = BeautifulSoup(response.content, "html.parser")

            for tag in soup.find_all("a", href=True):
                full_url = urljoin(url, tag["href"])
                parsed = urlparse(full_url)

                if parsed.netloc == base_domain:
                    urls_to_visit.add(full_url)

        except Exception as e:
            logging.warning(f"Failed to crawl {url}: {e}")

    logging.info(f"Crawl complete. Found {len(visited_urls)} URLs.")
    return visited_urls


# --------------------------------------------------
# TEXT EXTRACTION
# --------------------------------------------------

def extract_text_from_url(url: str):
    try:
        response = _robust_requests_get(url)
        soup = BeautifulSoup(response.content, "html.parser")

        article = soup.find("article")
        if article:
            return article.get_text(separator="\n", strip=True)

        logging.warning(f"No <article> found for {url}, using <body>")
        return soup.body.get_text(separator="\n", strip=True)

    except Exception as e:
        logging.error(f"Failed to extract {url}: {e}")
        return None


# --------------------------------------------------
# CHUNKING
# --------------------------------------------------

def _chunk_text_processor(text: str, chunk_size=1024, overlap=128):
    if not text:
        return []

    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunks.append(text[start:end])
        start += chunk_size - overlap

    return chunks


# --------------------------------------------------
# EMBEDDINGS
# --------------------------------------------------

@retry(stop=stop_after_attempt(3), wait=wait_fixed(2))
def embed(texts: list[str], client: cohere.Client):
    if not texts:
        return []

    response = client.embed(
        texts=texts,
        model="embed-english-v3.0",
        input_type="search_document",
    )
    return response.embeddings


# --------------------------------------------------
# QDRANT SETUP
# --------------------------------------------------

def create_collection(client: qdrant_client.QdrantClient):
    if client.collection_exists(COLLECTION_NAME):
        logging.info("Deleting existing collection")
        client.delete_collection(COLLECTION_NAME)

    client.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=EMBEDDING_SIZE,
            distance=Distance.COSINE,
        ),
    )

    client.create_payload_index(
        collection_name=COLLECTION_NAME,
        field_name="source_url",
        field_schema="keyword",
    )

    logging.info("Qdrant collection ready")


# --------------------------------------------------
# QDRANT OPERATIONS
# --------------------------------------------------

def save_chunk_to_qdrant(client, chunk_data):
    embedding = chunk_data["embedding"]

    # 🔒 HARD VALIDATION
    if not isinstance(embedding, list):
        raise ValueError("Embedding is not a list")

    if len(embedding) != EMBEDDING_SIZE:
        raise ValueError(f"Embedding size mismatch: {len(embedding)}")

    client.upsert(
        collection_name=COLLECTION_NAME,
        points=[
            PointStruct(
                id=chunk_data["id"],
                vector=embedding,
                payload={
                    "text": chunk_data["text"],
                    "source_url": chunk_data["metadata"]["source_url"],
                    "checksum": chunk_data["metadata"]["checksum"],
                },
            )
        ],
        wait=True,
    )


def get_existing_chunks(client, url: str):
    points, _ = client.scroll(
        collection_name=COLLECTION_NAME,
        scroll_filter=Filter(
            must=[
                FieldCondition(
                    key="source_url",
                    match=MatchValue(value=url),
                )
            ]
        ),
        limit=1000,
    )

    return {p.payload["checksum"]: p.id for p in points}


# --------------------------------------------------
# MAIN PIPELINE
# --------------------------------------------------

def main():
    logging.info("Starting ingestion pipeline")

    base_url = "https://physical-ai-humanoid-robotics-textb-eta.vercel.app/"

    co = cohere.Client(COHERE_API_KEY)
    # Validate Qdrant configuration and try to connect (with retries).
    def _validate_env():
        missing = []
        if not QDRANT_URL:
            missing.append("QDRANT_URL")
        if not QDRANT_API_KEY:
            missing.append("QDRANT_API_KEY")
        if missing:
            logging.error(
                "Missing environment variables: %s. Please set them in backend/.env",
                ",".join(missing),
            )
            sys.exit(1)

    _validate_env()

    @retry(stop=stop_after_attempt(3), wait=wait_fixed(2))
    def _connect_qdrant(url: str, api_key: str):
        client = qdrant_client.QdrantClient(url=url, api_key=api_key, timeout=60) # Increased timeout
        # simple health check: retrieve collections to verify connectivity
        client.get_collections()
        return client

    try:
        qdrant = _connect_qdrant(QDRANT_URL, QDRANT_API_KEY)
    except RetryError as e:
        logging.error("Unable to connect to Qdrant after several attempts: %s", e)
        logging.error(
            "Check backend/.env for QDRANT_URL and QDRANT_API_KEY, network/firewall, or run a local Qdrant instance."
        )
        logging.error("Example: docker run -p 6333:6333 qdrant/qdrant")
        sys.exit(1)

    create_collection(qdrant)

    all_urls = get_all_urls(base_url)
    logging.info(f"Processing {len(all_urls)} URLs")

    for url in all_urls:
        logging.info(f"Processing: {url}")

        existing_chunks = get_existing_chunks(qdrant, url)
        text = extract_text_from_url(url)

        if not text:
            if existing_chunks:
                qdrant.delete(
                    collection_name=COLLECTION_NAME,
                    points_selector=PointsSelector(
                        points=list(existing_chunks.values())
                    ),
                )
            continue

        chunks = _chunk_text_processor(text)
        checksums = {hashlib.sha256(c.encode()).hexdigest() for c in chunks}

        new_checksums = checksums - set(existing_chunks.keys())
        deleted_checksums = set(existing_chunks.keys()) - checksums

        new_texts = [
            c for c in chunks
            if hashlib.sha256(c.encode()).hexdigest() in new_checksums
        ]

        if new_texts:
            embeddings = embed(new_texts, co)

            for i, chunk_text_in_loop in enumerate(new_texts):
                checksum = hashlib.sha256(chunk_text_in_loop.encode()).hexdigest()
                # Create a unique UUID for the Qdrant point ID
                point_id = str(uuid.uuid4())

                chunk_data = {
                    "id": point_id,
                    "text": chunk_text_in_loop,
                    "embedding": embeddings[i],
                    "metadata": {
                        "source_url": url,
                        "checksum": checksum,
                    },
                }

                save_chunk_to_qdrant(qdrant, chunk_data)

        if deleted_checksums:
            qdrant.delete(
                collection_name=COLLECTION_NAME,
                points_selector=PointsSelector(
                    points=[existing_chunks[c] for c in deleted_checksums]
                ),
            )

    logging.info("Ingestion completed successfully")


if __name__ == "__main__":
    main()