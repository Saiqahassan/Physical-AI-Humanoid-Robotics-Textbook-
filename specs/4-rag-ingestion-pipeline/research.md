# Research: Ingestion Pipeline URL Crawling Strategy

**Date**: 2025-12-22

## Decision

The pipeline will use a recursive crawling strategy to discover all URLs within the target Docusaurus site.

## Rationale

The target site, `https://physical-ai-humanoid-robotics-textb-eta.vercel.app/`, is a modern web application (likely a Next.js SPA) and does not expose a `sitemap.xml` file. A dynamic, recursive crawling approach is therefore required to discover all internal pages.

The process will be as follows:
1.  **Initialization**: Start with a queue containing only the base URL and a set to store visited URLs.
2.  **Crawling Loop**:
    - Dequeue a URL and fetch its HTML content.
    - Add the URL to the visited set.
    - Parse the HTML to extract all `<a>` tags.
3.  **Link Filtering**:
    - For each extracted link, resolve it to an absolute URL.
    - If the URL belongs to the same origin as the base URL and has not been visited yet, add it to the queue.
4.  **Termination**: The loop continues until the queue is empty.

This ensures comprehensive coverage of all reachable pages within the site.

## Alternatives Considered

- **Sitemap Parsing**: A `sitemap.xml` was not found on the target site.
- **Static List of URLs**: This is not scalable and would require manual updates whenever the site content changes.
