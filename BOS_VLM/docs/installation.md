# Installation Guide

## Prerequisites

- Python 3.8 or higher
- Pip package manager
- Git

## Installation Steps

### 1. Clone the Repository

\\\ash
git clone https://github.com/homebrewroboticsclub/BOS_VLM.git
cd BOS_VLM
\\\

### 2. Install Dependencies

\\\ash
pip install -e .
\\\

This will install all required Python packages.

### 3. Download Pre-trained Models

\\\ash
# Download only CLIP model
./scripts/download_models.sh

# Download all supported models (CLIP and OWL-ViT)
./scripts/download_models.sh all
\\\

### 4. Build BOS Package

\\\ash
# Create symbolic link to your Brew workspace
ln -s /path/to/BOS_VLM ~/brew_ws/src/

# Build the package
cd ~/brew_ws
make
\\\

## Docker Installation

We also provide a Docker container for easier installation:

\\\ash
# Build the Docker image
docker build -t bos_vlm .

# Run the container
docker run -it --gpus all bos_vlm
\\\

## Troubleshooting

If you encounter issues with model downloads, make sure you have an active internet connection and sufficient disk space.

For CUDA-related issues, ensure that your CUDA drivers are properly installed and compatible with the PyTorch version.
