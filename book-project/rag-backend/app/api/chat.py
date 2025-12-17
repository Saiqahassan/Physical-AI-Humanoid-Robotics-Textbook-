from fastapi import APIRouter, Depends
from pydantic import BaseModel
from typing import List, Optional
from app.services.rag_service import RAGService
from app.core.vector_db import get_qdrant_client
from qdrant_client import QdrantClient

router = APIRouter()

class ChatRequest(BaseModel):
    query: str
    session_id: Optional[str] = None
    user_id: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    sources: List[str]
    session_id: str
    timestamp: str

@router.post("/chat", response_model=ChatResponse)
async def chat_with_rag(
    request: ChatRequest,
    qdrant_client: QdrantClient = Depends(get_qdrant_client)
):
    """
    Send a query to the RAG chatbot and get a response.
    """
    rag_service = RAGService(qdrant_client)

    # Placeholder for embedding generation (will be implemented in a later task)
    # For now, we simulate a query vector
    query_vector = [0.1] * 768 # Dummy vector of size 768

    retrieved_docs = await rag_service.retrieve_relevant_documents(query_vector)

    # In a real scenario, this would use an LLM
    answer = await rag_service.generate_answer(request.query, [doc for doc in retrieved_docs])

    return ChatResponse(
        answer=answer,
        sources=[], # Placeholder for sources
        session_id=request.session_id or "default_session",
        timestamp="2025-12-07T00:00:00Z" # Placeholder
    )
