import cohere
import qdrant_client
import logging
from qdrant_client.http import models
from qdrant_client.http.exceptions import UnexpectedResponse
from typing import List, Optional, Dict

from backend.core.config import get_settings
from backend.retrieval.data_models import RetrievedChunk

def retrieve_chunks(
    query: str,
    filters: Optional[Dict[str, str]] = None,
    top_k: int = 5
) -> List[RetrievedChunk]:
    """
    Retrieves the most relevant document chunks from Qdrant based on a query.
    
    Raises:
        cohere.CohereAPIError: If there is an issue with the Cohere API.
        qdrant_client.http.exceptions.UnexpectedResponse: If Qdrant returns an
            unexpected response (e.g., validation error).
        ConnectionRefusedError: If the Qdrant service is not reachable.
    """
    try:
        settings = get_settings()
        co = cohere.Client(settings.COHERE_API_KEY)
        qdrant = qdrant_client.QdrantClient(
            url=settings.QDRANT_URL, 
            api_key=settings.QDRANT_API_KEY
        )

        # Embed the query
        response = co.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        query_embedding = response.embeddings[0]

        # Construct the Qdrant filter
        qdrant_filter = None
        if filters:
            qdrant_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key=key,
                        match=models.MatchValue(value=value)
                    ) for key, value in filters.items()
                ]
            )

        # Perform the search
        search_result = qdrant.search(
            collection_name="rag_embedding",
            query_vector=query_embedding,
            query_filter=qdrant_filter,
            limit=top_k,
            with_payload=True
        )

        # Process results
        retrieved_chunks = [
            RetrievedChunk(
                text=point.payload["text"],
                source_url=point.payload["source_url"],
                score=point.score
            ) for point in search_result
        ]

        return retrieved_chunks

    except (cohere.CohereAPIError, UnexpectedResponse, ConnectionRefusedError) as e:
        logging.error(f"An error occurred during retrieval: {e}")
        raise
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}")
        raise
