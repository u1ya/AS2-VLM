#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export OLLAMA_HOST=127.0.0.1:11434
export OLLAMA_MODELS="$project_root/.models/ollama"
command -v ollama >/dev/null || { printf '%s\n' 'Install Ollama from https://ollama.com/download first.' >&2; exit 1; }
exec ollama serve
