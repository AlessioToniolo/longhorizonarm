import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from perception.ml.vlm_handler import VLMHandler
import cv2

def draw_detection(frame, result):
    # Extract bbox coordinates
    x1, y1, x2, y2 = map(int, result['box'])  # Changed from 'bbox' to 'box'
    label = result['label']
    score = result['score']

    # Draw rectangle
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    # Add label with score
    text = f"{label}: {score:.2f}"
    cv2.putText(frame, text, (x1, y1-10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

def main():
    vlm_handler = VLMHandler()
    try:
        while True:
            frame, results = vlm_handler.process_frame()
            if frame is not None:
                # Draw bounding boxes for each detection
                if results:
                    for result in results:
                        draw_detection(frame, result)
                
                cv2.imshow('RealSense', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        vlm_handler.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()