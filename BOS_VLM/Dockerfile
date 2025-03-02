FROM ubuntu:22.04

# Set environment variables
ENV PYTHONUNBUFFERED=1 \
    DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    libopencv-dev \
    python3-opencv \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace
RUN mkdir -p /brew_ws/src
WORKDIR /brew_ws/src

# Clone the repository (this will be mounted in development)
RUN mkdir -p BOS_VLM

# Install Python dependencies
COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Install PyTorch with CUDA support
# Uncomment the appropriate line based on your CUDA version
# For CUDA 11.8
RUN pip3 install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
# For CUDA 12.1
# RUN pip3 install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
# For CPU only
# RUN pip3 install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# Install Transformers and other ML libraries
RUN pip3 install --no-cache-dir \
    transformers \
    timm \
    huggingface-hub \
    pillow \
    einops \
    opencv-python-headless

# Set up the workspace
WORKDIR /brew_ws

# Download model weights (optional, can also be done at runtime)
WORKDIR /brew_ws/src/BOS_VLM
RUN mkdir -p data/models

# Set the working directory
WORKDIR /brew_ws

# entrypoint
COPY ./docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
