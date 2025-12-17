from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
import os

QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
QDRANT_PORT = int(os.getenv("QDRANT_PORT", 6333))
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_content")
VECTOR_SIZE = int(os.getenv("VECTOR_SIZE", 768)) # Example size, should match embedding model

def get_qdrant_client():
    """Initializes and returns a Qdrant client."""
    client = QdrantClient(host=QDRANT_HOST, port=QDRANT_PORT)
    return client

def create_collection_if_not_exists(client: QdrantClient):
    """Creates the Qdrant collection if it doesn't already exist."""
    collections = client.get_collections().collections
    if not any(c.name == QDRANT_COLLECTION_NAME for c in collections):
        client.recreate_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=VectorParams(size=VECTOR_SIZE, distance=Distance.COSINE),
        )
