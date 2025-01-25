import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from perception.ml.vlm_handler import VLMHandler
import cv2

def main():
    vlm_handler = VLMHandler()

    try:
        while True:
            frame, results = vlm_handler.process_frame()
            if frame is not None:
                # Display the camera stream
                cv2.imshow('RealSense', frame)
                
                # Print detected objects to terminal
                if results:
                    print("\nDetected Objects:")
                    for result in results:
                        print(f"Label: {result['label']}, Score: {result['score']:.2f}")
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        vlm_handler.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()