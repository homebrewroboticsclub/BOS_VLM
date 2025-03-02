#!/bin/bash
set -e

# Source Brew setup
source /opt/brew/setup.bash

# Build the workspace if the package exists
if [ -d "/brew_ws/src/BOS_VLM" ]; then
    cd /brew_ws
    make
    source /brew_ws/devel/setup.bash
fi

# Download model weights if they don't exist
MODELS_DIR="/brew_ws/src/BOS_VLM/data/models"
if [ ! -d "/clip" ]; then
    echo "Downloading CLIP model weights..."
    mkdir -p "/clip"
    python3 -c "from transformers import CLIPProcessor, CLIPModel; model = CLIPModel.from_pretrained('openai/clip-vit-base-patch32', cache_dir='/clip')"
fi

# Execute the command passed to the container
exec "$@"
