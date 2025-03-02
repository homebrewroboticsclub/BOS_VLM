"""
Text processing utilities for BOS_VLM.
"""
import re
from typing import List, Optional

def clean_text(text: str) -> str:
    """
    Clean text by removing extra whitespace and special characters.
    
    Args:
        text: Input text
        
    Returns:
        Cleaned text
    """
    # Remove extra whitespace
    text = re.sub(r'\s+', ' ', text).strip()
    
    # Remove special characters
    text = re.sub(r'[^\w\s.,?!]', '', text)
    
    return text

def tokenize(text: str, max_length: Optional[int] = None) -> List[str]:
    """
    Split text into tokens.
    
    Args:
        text: Input text
        max_length: Maximum number of tokens to return
        
    Returns:
        List of tokens
    """
    # Simple whitespace tokenization
    tokens = text.split()
    
    # Apply max length if specified
    if max_length is not None:
        tokens = tokens[:max_length]
    
    return tokens

def format_prompt(template: str, **kwargs) -> str:
    """
    Format a prompt template with variables.
    
    Args:
        template: Prompt template with {variable} placeholders
        **kwargs: Variables to fill in the template
        
    Returns:
        Formatted prompt
    """
    return template.format(**kwargs)
