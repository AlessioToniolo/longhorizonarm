from transformers import AutoModelForObjectDetection, DetrImageProcessor
import torch
import numpy as np
import v2.camera as Camera
import cv2

class VLM:
    def __init__(self, stream = None):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")
        self.model = AutoModelForObjectDetection.from_pretrained("facebook/detr-resnet-50").to(self.device)
        if stream == None:
            self.realsense = Camera()
        else:
            self.realsense = stream

    def predict(self):
        frame = self.realsense.get_frame()
        processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_rgb = processed_frame[..., ::-1]
        inputs = self.processor(images=frame_rgb, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        with torch.no_grad():
            outputs = self.model(**inputs)
        
        target_sizes = torch.tensor([processed_frame.shape[:2]]).to(self.device)
        results = self.processor.post_process_object_detection(
            outputs, 
            target_sizes=target_sizes,
            threshold=0.5
        )[0]
        
        predictions = []
        for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
            predictions.append({
                'box': box.cpu().numpy(),
                'score': score.cpu().numpy(),
                'label': self.model.config.id2label[label.item()]
            })
            
        return predictions
    
    