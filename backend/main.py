import logging
import os
import hashlib
from urllib.parse import urljoin, urlparse
from tenacity import retry, stop_after_attempt, wait_fixed

# --- Setup Logging ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

import cohere
import qdrant_client
import requests
from bs4 import BeautifulSoup
from dotenv import load_dotenv

load_dotenv()

# --- Configuration ---
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
COLLECTION_NAME = "rag_embedding"

@retry(stop=stop_after_attempt(3), wait=wait_fixed(2))
def _robust_requests_get(url: str):
    response = requests.get(url, timeout=5)
    response.raise_for_status()
    return response

def get_all_urls(base_url: str):
    """
    Crawls a base URL to find all unique, same-origin URLs.
    """
    logging.info(f"Starting crawl from base URL: {base_url}")
    base_domain = urlparse(base_url).netloc
    urls_to_visit = {base_url}
    visited_urls = set()

    while urls_to_visit:
        url = urls_to_visit.pop()
        if url in visited_urls:
            continue
        
        logging.info(f"Crawling: {url}")
        visited_urls.add(url)

        try:
            response = _robust_requests_get(url)
            soup = BeautifulSoup(response.content, "html.parser")

            for a_tag in soup.find_all("a", href=True):
                href = a_tag["href"]
                full_url = urljoin(url, href)
                parsed_full_url = urlparse(full_url)

                if parsed_full_url.netloc == base_domain and full_url not in visited_urls:
                    urls_to_visit.add(full_url)
        except Exception as e: # Catch all exceptions from retry attempts
            logging.warning(f"Could not fetch {url} after multiple retries: {e}")

    logging.info(f"Crawl complete. Found {len(visited_urls)} URLs.")
    return visited_urls


def extract_text_from_url(url: str):
    """
    Extracts the main text content from a given URL.
    """
    try:
        response = _robust_requests_get(url)
        soup = BeautifulSoup(response.content, "html.parser")

        # Docusaurus sites often have their main content in an <article> tag
        main_content = soup.find("article")
        if main_content:
            return main_content.get_text(separator="\\n", strip=True)
        else:
            logging.warning(f"No <article> tag found for {url}. Falling back to body.")
            # Fallback for pages without a clear article structure
            return soup.body.get_text(separator="\\n", strip=True)
    except Exception as e: # Catch all exceptions from retry attempts
        logging.error(f"Could not fetch or read {url} after multiple retries: {e}")
        return None


def chunk_text(text: str, chunk_size: int = 1024, overlap: int = 128):
    """
    Splits text into overlapping chunks.
    """
    if not text:
        return []
    
    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunks.append(text[start:end])
        start += chunk_size - overlap
    return chunks


@retry(stop=stop_after_attempt(3), wait=wait_fixed(2))
def embed(texts: list[str], co_client: cohere.Client):
    """
    Generates embeddings for a list of texts using Cohere.
    """
    if not texts:
        return []

    try:
        response = co_client.embed(
            texts=texts,
            model="embed-english-v3.0",
            input_type="search_document"
        )
        return response.embeddings
    except cohere.CohereError as e:
        logging.error(f"Cohere API error: {e} after multiple retries.")
        return []

def create_collection(client: qdrant_client.QdrantClient):
    """
    Creates the Qdrant collection if it doesn't exist.
    """
    try:
        client.get_collection(collection_name=COLLECTION_NAME)
        logging.info(f"Collection '{COLLECTION_NAME}' already exists.")
    except Exception:
        client.recreate_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=qdrant_client.http.models.VectorParams(
                size=1024, distance=qdrant_client.http.models.Distance.COSINE
            ),
        )
        logging.info(f"Collection '{COLLECTION_NAME}' created.")

def save_chunk_to_qdrant(client: qdrant_client.QdrantClient, chunk_data: dict):
    """
    Saves a single chunk and its embedding to Qdrant.
    """
    client.upsert(
        collection_name=COLLECTION_NAME,
        points=[
            qdrant_client.http.models.PointStruct(
                id=chunk_data["id"],
                vector=chunk_data["embedding"],
                payload={
                    "text": chunk_data["text"],
                    "source_url": chunk_data["metadata"]["source_url"],
                    "checksum": chunk_data["metadata"]["checksum"],
                },
            )
        ],
        wait=True,
    )

def get_existing_chunks(client: qdrant_client.QdrantClient, url: str):
    """
    Retrieves existing chunks for a given URL from Qdrant.
    """
    response, _ = client.scroll(
        collection_name=COLLECTION_NAME,
        scroll_filter=qdrant_client.http.models.Filter(
            must=[
                qdrant_client.http.models.FieldCondition(
                    key="source_url",
                    match=qdrant_client.http.models.MatchValue(value=url),
                )
            ]
        ),
        limit=1000,  # Adjust limit as needed
    )
    return {point.payload["checksum"]: point.id for point in response}

def main():
    """
    Main function to execute the ingestion pipeline.
    """
    logging.info("Starting ingestion pipeline.")
    base_url = "https://physical-ai-humanoid-robotics-textb-eta.vercel.app/"

    # 1. Initialize clients
    co = cohere.Client(COHERE_API_KEY)
    qdrant = qdrant_client.QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

    # 2. Create collection if it doesn't exist
    create_collection(qdrant)

    # 3. Crawl for all URLs
    all_urls = get_all_urls(base_url)
    if not all_urls:
        logging.warning("No URLs found to process. Exiting.")
        return

    logging.info(f"Found {len(all_urls)} URLs to process.")

    # 4. Process each URL
    for url in all_urls:
        logging.info(f"Processing: {url}")
        
        existing_chunks = get_existing_chunks(qdrant, url)
        
        text = extract_text_from_url(url)
        if not text:
            if existing_chunks:
                logging.info(f"Content disappeared from {url}. Deleting old chunks.")
                qdrant.delete(collection_name=COLLECTION_NAME, points_selector=list(existing_chunks.values()))
            continue

        current_chunks = chunk_text(text)
        current_checksums = {hashlib.sha256(chunk.encode()).hexdigest() for chunk in current_chunks}

        new_checksums = current_checksums - set(existing_chunks.keys())
        deleted_checksums = set(existing_chunks.keys()) - current_checksums

        new_texts = [chunk for chunk in current_chunks if hashlib.sha256(chunk.encode()).hexdigest() in new_checksums]
        
        if new_texts:
            logging.info(f"Found {len(new_texts)} new or updated chunks for {url}.")
            embeddings = embed(new_texts, co)
            if embeddings:
                for i, chunk_text in enumerate(new_texts):
                    chunk_hash = hashlib.sha256(chunk_text.encode()).hexdigest()
                    chunk_data = {
                        "id": f"{url}#{chunk_hash}",
                        "text": chunk_text,
                        "embedding": embeddings[i],
                        "metadata": {"source_url": url, "checksum": chunk_hash},
                    }
                    save_chunk_to_qdrant(qdrant, chunk_data)

        if deleted_checksums:
            logging.info(f"Found {len(deleted_checksums)} chunks to delete for {url}.")
            points_to_delete = [existing_chunks[checksum] for checksum in deleted_checksums]
            qdrant.delete(collection_name=COLLECTION_NAME, points_selector=points_to_delete)

    logging.info("Ingestion pipeline completed.")


if __name__ == "__main__":
    main()
