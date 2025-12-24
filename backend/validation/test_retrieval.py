import os
import logging
from dotenv import load_dotenv
from backend.retrieval.main import retrieve_chunks

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def main():
    """
    Example of how to call the retrieval service.
    """
    # Load environment variables from .env file in the parent directory
    dotenv_path = os.path.join(os.path.dirname(__file__), '..', '.env')
    if not os.path.exists(dotenv_path):
        logging.error(f".env file not found at {dotenv_path}. Please create one based on .env.example.")
        return
    load_dotenv(dotenv_path=dotenv_path)

    query = "What is the role of a state estimator in robotics?"

    logging.info(f"Searching for: '{query}'")

    try:
        # Basic retrieval
        results = retrieve_chunks(query=query, top_k=3)

        logging.info(f"Found {len(results)} results:")
        for i, chunk in enumerate(results):
            logging.info(f"  {i+1}. Score: {chunk.score:.4f}, Source: {chunk.source_url}")

        # Retrieval with metadata filter
        logging.info("\nSearching with a filter...")
        filters = {"source_url": "https://physical-ai-humanoid-robotics-textb-eta.vercel.app/docs/intro"}
        filtered_results = retrieve_chunks(query=query, filters=filters, top_k=2)

        logging.info(f"Found {len(filtered_results)} filtered results:")
        for i, chunk in enumerate(filtered_results):
            logging.info(f"  {i+1}. Score: {chunk.score:.4f}, Source: {chunk.source_url}")

    except Exception as e:
        logging.error(f"An error occurred during the test run: {e}", exc_info=True)

if __name__ == "__main__":
    main()
