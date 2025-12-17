from fastapi import APIRouter

router = APIRouter()

@router.get("/health", response_model=dict)
async def get_health():
    """
    Health check endpoint.
    """
    return {"status": "healthy", "version": "1.0.0"}
