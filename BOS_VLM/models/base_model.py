"""
Base class for visual language models to be used with BOS.
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple, Union
import numpy as np


class BaseVLM(ABC):
    """Abstract base class for all visual language models."""
    
    def __init__(self, config: Dict = None):
        """
        Initialize the visual language model.
        
        Args:
            config: Configuration dictionary for the model
        """
        self.config = config or {}
        self.model = None
        self.processor = None
        self.device = self.config.get('device', 'cuda' if self.is_cuda_available() else 'cpu')
        
    @staticmethod
    def is_cuda_available() -> bool:
        """Check if CUDA is available."""
        try:
            import torch
            return torch.cuda.is_available()
        except ImportError:
            return False
    
    @abstractmethod
    def load_model(self) -> None:
        """Load the model and processor."""
        pass
    
    @abstractmethod
    def process_image(self, image: np.ndarray) -> any:
        """
        Process an image for model input.
        
        Args:
            image: RGB image as numpy array (H, W, 3)
            
        Returns:
            Processed image in the format required by the model
        """
        pass
    
    @abstractmethod
    def process_text(self, text: str) -> any:
        """
        Process text for model input.
        
        Args:
            text: Input text
            
        Returns:
            Processed text in the format required by the model
        """
        pass
        
    @abstractmethod
    def run_inference(self, 
                     image: np.ndarray, 
                     text: Optional[str] = None,
                     **kwargs) -> Dict:
        """
        Run inference with the model.
        
        Args:
            image: RGB image as numpy array (H, W, 3)
            text: Optional text query
            **kwargs: Additional model-specific parameters
            
        Returns:
            Dictionary containing model outputs
        """
        pass
    
    def describe_image(self, image: np.ndarray) -> str:
        """
        Generate a description of the image.
        
        Args:
            image: RGB image as numpy array (H, W, 3)
            
        Returns:
            Text description of the image
        """
        return self.run_inference(image, task="describe")["description"]
    
    def answer_question(self, image: np.ndarray, question: str) -> str:
        """
        Answer a question about the image.
        
        Args:
            image: RGB image as numpy array (H, W, 3)
            question: Question about the image
            
        Returns:
            Answer to the question
        """
        return self.run_inference(image, text=question, task="vqa")["answer"]
    
    def detect_objects(self, 
                      image: np.ndarray, 
                      queries: List[str] = None) -> Dict[str, List[Tuple[float, float, float, float]]]:
        """
        Detect objects in the image.
        
        Args:
            image: RGB image as numpy array (H, W, 3)
            queries: List of object categories to detect
            
        Returns:
            Dictionary mapping object categories to bounding boxes
            Format: {category: [(x1, y1, x2, y2, score), ...]}
        """
        return self.run_inference(image, text=queries, task="detection")["detections"]
    
    def embed_image(self, image: np.ndarray) -> np.ndarray:
        """
        Generate an embedding for the image.
        
        Args:
            image: RGB image as numpy array (H, W, 3)
            
        Returns:
            Image embedding as numpy array
        """
        return self.run_inference(image, task="embed")["embedding"]
    
    def embed_text(self, text: str) -> np.ndarray:
        """
        Generate an embedding for the text.
        
        Args:
            text: Input text
            
        Returns:
            Text embedding as numpy array
        """
        # For models that support text embedding
        if hasattr(self, "text_encoder"):
            return self.run_inference(None, text=text, task="embed_text")["embedding"]
        else:
            raise NotImplementedError(f"{self.__class__.__name__} does not support text embedding")
# End of file
