from typing import Optional, Dict, List
from pydantic import BaseModel, Field

class RetrievalRequest(BaseModel):
    """
    Represents the input to the retrieval service.
    """
    query: str = Field(..., min_length=1)
    filters: Optional[Dict[str, str]] = None
    top_k: int = Field(5, ge=1, le=25)

class RetrievedChunk(BaseModel):
    """
    Represents a single retrieved document chunk.
    """
    text: str
    source_url: str
    score: float
