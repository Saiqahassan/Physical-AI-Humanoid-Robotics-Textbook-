from fastapi import FastAPI
from app.api import health, chat

app = FastAPI()

app.include_router(health.router)
app.include_router(chat.router)

@app.get("/")
async def root():
    return {"message": "FastAPI RAG Backend is running!"}
