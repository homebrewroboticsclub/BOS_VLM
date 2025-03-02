"""
CLIP model implementation for BOS_VLM.
"""

import numpy as np
from typing import Dict, List, Optional, Tuple, Union
import torch
from PIL import Image

from BOS_VLM.models.base_model import BaseVLM


class CLIPModel(BaseVLM):
    """
    Implementation of CLIP (Contrastive Language-Image Pre-Training) model.
    """
    
    def __init__(self, config: Dict = None):
        """
        Initialize the CLIP model.
        
        Args:
            config: Configuration dictionary with the following options:
                - model_name: Name of the CLIP model (default: "openai/clip-vit-base-patch32")
                - device: Device to run the model on (default: "cuda" if available, else "cpu")
                - cache_dir: Directory to cache the model
        """
        super().__init__(config)
        self.model_name = self.config.get("model_name", "openai/clip-vit-base-patch32")
        self.cache_dir = self.config.get("cache_dir", None)
        self.load_model()
        
    def load_model(self) -> None:
        """Load the CLIP model and processor."""
        try:
            from transformers import CLIPProcessor, CLIPModel
        except ImportError:
            raise ImportError(
                "Could not import transformers. "
                "Please install it with pip install transformers."
            )
            
        self.model = CLIPModel.from_pretrained(
            self.model_name, 
            cache_dir=self.cache_dir
        ).to(self.device)
        
        self.processor = CLIPProcessor.from_pretrained(
            self.model_name,
            cache_dir=self.cache_dir
        )
        
    def process_image(self, image: np.ndarray) -> torch.Tensor:
        """
        Process an image for CLIP input.
        
        Args:
            image: RGB image as numpy array (H, W, 3)
            
        Returns:
            Processed image as torch.Tensor
        """
        # Convert numpy array to PIL Image
        if isinstance(image, np.ndarray):
            image = Image.fromarray(image.astype('uint8'))
            
        # Process the image
        inputs = self.processor(
            images=image,
            return_tensors="pt",
            padding=True
        )
        
        return inputs.pixel_values.to(self.device)
    
    def process_text(self, text: Union[str, List[str]]) -> torch.Tensor:
        """
        Process text for CLIP input.
        
        Args:
            text: Input text or list of texts
            
        Returns:
            Processed text as torch.Tensor
        """
        # Handle both single string and list of strings
        if isinstance(text, str):
            text = [text]
            
        # Process the text
        inputs = self.processor(
            text=text,
            return_tensors="pt",
            padding=True,
            truncation=True
        )
        
        return inputs.input_ids.to(self.device)
        
    def run_inference(self, 
                     image: Optional[np.ndarray] = None, 
                     text: Optional[Union[str, List[str]]] = None,
                     task: str = "similarity",
                     **kwargs) -> Dict:
        """
        Run inference with the CLIP model.
        
        Args:
            image: RGB image as numpy array (H, W, 3)
            text: Text query or list of text queries
            task: Type of inference to run:
                - "similarity": Compute similarity between image and text
                - "embed": Generate image embedding
                - "embed_text": Generate text embedding
            **kwargs: Additional model-specific parameters
            
        Returns:
            Dictionary containing model outputs
        """
        results = {}
        
        # Put model in evaluation mode
        self.model.eval()
        
        with torch.no_grad():
            if task == "similarity" and image is not None and text is not None:
                # Process inputs
                image_input = self.process_image(image)
                text_input = self.process_text(text)
                
                # Get image and text features
                outputs = self.model(input_ids=text_input, pixel_values=image_input)
                
                # Normalize features
                image_embeds = outputs.image_embeds / outputs.image_embeds.norm(dim=-1, keepdim=True)
                text_embeds = outputs.text_embeds / outputs.text_embeds.norm(dim=-1, keepdim=True)
                
                # Compute similarity scores
                logits_per_image = (100.0 * image_embeds @ text_embeds.T).softmax(dim=-1)
                probs = logits_per_image.cpu().numpy()
                
                results["scores"] = probs[0]
                
                # If text is a list, return the most similar text
                if isinstance(text, list):
                    best_idx = probs[0].argmax()
                    results["best_match"] = {
                        "text": text[best_idx],
                        "score": float(probs[0][best_idx])
                    }
            
            elif task == "embed" and image is not None:
                # Process image
                image_input = self.process_image(image)
                
                # Get image features
                outputs = self.model.get_image_features(pixel_values=image_input)
                
                # Normalize features
                image_embeds = outputs / outputs.norm(dim=-1, keepdim=True)
                
                results["embedding"] = image_embeds.cpu().numpy()[0]
                
            elif task == "embed_text" and text is not None:
                # Process text
                text_input = self.process_text(text)
                
                # Get text features
                outputs = self.model.get_text_features(input_ids=text_input)
                
                # Normalize features
                text_embeds = outputs / outputs.norm(dim=-1, keepdim=True)
                
                results["embedding"] = text_embeds.cpu().numpy()[0]
            
            else:
                raise ValueError(
                    f"Invalid task '{task}' or missing required inputs. "
                    f"Task '{task}' requires image={image is not None} and text={text is not None}."
                )
                
        return results
    
    def find_best_match(self, image: np.ndarray, candidates: List[str]) -> Tuple[str, float]:
        """
        Find the text that best matches the image from a list of candidates.
        
        Args:
            image: RGB image as numpy array (H, W, 3)
            candidates: List of text candidates
            
        Returns:
            Tuple of (best matching text, score)
        """
        results = self.run_inference(image, candidates, task="similarity")
        return results["best_match"]["text"], results["best_match"]["score"]
