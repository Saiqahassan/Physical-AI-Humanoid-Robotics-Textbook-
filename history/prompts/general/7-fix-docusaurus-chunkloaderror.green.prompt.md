---
id: 7
title: Fix Docusaurus ChunkLoadError
stage: green
date: 2025-12-16
surface: agent
model: gemini-pro
feature: none
branch: main
user: placeholder
command: ChunkLoadError
labels: ["bugfix", "docusaurus", "webpack"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - book-project/docusaurus-site/.docusaurus
 - book-project/docusaurus-site/build
tests:
 - npm start
---

## Prompt

Loading chunk vendors-node_modules_docusaurus_theme-classic_lib_theme_EditMetaRow_index_js-node_modules_doc-fdb698 failed.(error: http://localhost:8081/vendors-node_modules_docusaurus_theme-classic_lib_theme_EditMetaRow_index_js-node_modules_doc-fdb698.js)ChunkLoadError    at __webpack_require__.f.j (http://localhost:8081/runtime~main.js:803:29)    at http://localhost:8081/runtime~main.js:146:40    at Array.reduce (<anonymous>)    at __webpack_require__.e (http://localhost:8081/runtime~main.js:145:67)    at fn.e (http://localhost:8081/runtime~main.js:341:50)    at __WEBPACK_DEFAULT_EXPORT__.__comp---theme-blog-list-pagea-6-a-7ba (webpack-internal:///./.docusaurus/registry.js:5:1044)    at load (webpack-internal:///./node_modules/react-loadable/lib/index.js:28:17)    at eval (webpack-internal:///./node_modules/react-loadable/lib/index.js:56:20)    at Array.forEach (<anonymous>)    at loadMap (webpack-internal:///./node_modules/react-loadable/lib/index.js:55:22)

## Response snapshot

[SUCCESS] Docusaurus website is running at: http://localhost:3001/

## Outcome

- ✅ Impact: The user's development server was broken and is now running successfully.
- 🧪 Tests: The fix was verified by successfully starting the development server.
- 📁 Files: Deleted cached build artifacts (`.docusaurus`, `build`) to resolve the issue.
- 🔁 Next prompts: The user may want to address the new markdown link warnings.
- 🧠 Reflection: The issue was a classic stale cache problem. The standard procedure of cleaning build artifacts and restarting the server was effective. The inability to read config files due to `.gitignore` was a minor obstacle but could be overcome by relying on standard project conventions.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): None
