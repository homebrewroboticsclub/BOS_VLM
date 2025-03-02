#!/bin/bash
# Script to download pre-trained models

# Create models directory
mkdir -p data/models

# Download CLIP model
echo "Downloading CLIP model..."
python3 -c "from transformers import CLIPProcessor, CLIPModel; model = CLIPModel.from_pretrained('openai/clip-vit-base-patch32', cache_dir='data/models/clip')"

# Download OWL-ViT model (if needed)
if [ "" == "all" ]; then
  echo "Downloading OWL-ViT model..."
  python3 -c "from transformers import OwlViTProcessor, OwlViTForObjectDetection; model = OwlViTForObjectDetection.from_pretrained('google/owlvit-base-patch32', cache_dir='data/models/owlvit')"
fi

echo "Model download complete!"
