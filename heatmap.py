#!/usr/bin/env python3
"""
Simple Heatmap Viewer - No Ignition Python bindings required!

This script reads heatmap data by subscribing via subprocess to ign topic,
then displays it using OpenCV.

Usage:
    python3 heatmap_viewer_simple.py

Requirements:
    pip install opencv-python numpy
"""

import subprocess
import numpy as np
import cv2
import re
import threading
import queue
import signal
import sys

class HeatmapViewer:
    def __init__(self, topic="/gnb/heatmap", width=128, height=128):
        self.topic = topic
        self.width = width
        self.height = height
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.process = None
        
    def parse_image_data(self, raw_text):
        """Parse the protobuf text format output from ign topic."""
        try:
            # Extract width and height
            width_match = re.search(r'width:\s*(\d+)', raw_text)
            height_match = re.search(r'height:\s*(\d+)', raw_text)
            
            if width_match and height_match:
                w = int(width_match.group(1))
                h = int(height_match.group(1))
            else:
                w, h = self.width, self.height
            
            # Extract the data field - it's between 'data: "' and the closing '"'
            # The data contains escaped octal sequences like \377
            data_match = re.search(r'data:\s*"(.*?)"(?:\s*pixel_format_type|\s*$)', 
                                   raw_text, re.DOTALL)
            
            if not data_match:
                return None
            
            raw_data = data_match.group(1)
            
            # Decode the escaped string (handles \377 octal, \x00 hex, etc.)
            try:
                decoded = raw_data.encode('utf-8').decode('unicode_escape').encode('latin-1')
            except:
                # Alternative decoding for complex escape sequences
                decoded = bytes(raw_data, 'utf-8').decode('unicode_escape').encode('latin-1')
            
            if len(decoded) != w * h * 3:
                print(f"Size mismatch: got {len(decoded)}, expected {w * h * 3}")
                return None
            
            # Convert to numpy array
            img = np.frombuffer(decoded, dtype=np.uint8).reshape((h, w, 3))
            
            # Convert RGB to BGR for OpenCV
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            
        except Exception as e:
            print(f"Parse error: {e}")
            return None
    
    def subscriber_thread(self):
        """Thread that runs ign topic and parses output."""
        cmd = ["ign", "topic", "-e", "-t", self.topic]
        
        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            
            buffer = ""
            in_message = False
            
            for line in self.process.stdout:
                if not self.running:
                    break
                
                buffer += line
                
                # Detect message boundaries (empty line or new 'width:' field)
                if 'pixel_format_type:' in line:
                    # End of message, parse it
                    img = self.parse_image_data(buffer)
                    if img is not None:
                        try:
                            self.image_queue.put_nowait(img)
                        except queue.Full:
                            # Drop old frame, add new one
                            try:
                                self.image_queue.get_nowait()
                                self.image_queue.put_nowait(img)
                            except:
                                pass
                    buffer = ""
                    
        except Exception as e:
            print(f"Subscriber error: {e}")
        finally:
            if self.process:
                self.process.terminate()
    
    def add_legend(self, img):
        """Add a color legend to the image."""
        h, w = img.shape[:2]
        legend_width = 30
        legend = np.zeros((h, legend_width, 3), dtype=np.uint8)
        
        for y in range(h):
            normalized = 1.0 - (y / h)
            
            if normalized < 0.25:
                t = normalized / 0.25
                r, g, b = 0, int(255 * t), 255
            elif normalized < 0.5:
                t = (normalized - 0.25) / 0.25
                r, g, b = 0, 255, int(255 * (1 - t))
            elif normalized < 0.75:
                t = (normalized - 0.5) / 0.25
                r, g, b = int(255 * t), 255, 0
            else:
                t = (normalized - 0.75) / 0.25
                r, g, b = 255, int(255 * (1 - t)), 0
            
            legend[y, :] = [b, g, r]
        
        # Add padding for text
        padding = np.zeros((h, 80, 3), dtype=np.uint8)
        combined = np.hstack([img, legend, padding])
        
        # Add text labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(combined, "-30 dBm", (w + 35, 20), font, 0.5, (255, 255, 255), 1)
        cv2.putText(combined, "(strong)", (w + 35, 40), font, 0.35, (200, 200, 200), 1)
        cv2.putText(combined, "-75 dBm", (w + 35, h // 2), font, 0.5, (255, 255, 255), 1)
        cv2.putText(combined, "-120 dBm", (w + 35, h - 20), font, 0.5, (255, 255, 255), 1)
        cv2.putText(combined, "(weak)", (w + 35, h), font, 0.35, (200, 200, 200), 1)
        
        return combined
    
    def run(self):
        """Main display loop."""
        print(f"Subscribing to {self.topic}...")
        print("Press 'q' to quit, 's' to save current frame")
        print("Make sure Gazebo is running with the heatmap plugin!")
        
        # Start subscriber thread
        sub_thread = threading.Thread(target=self.subscriber_thread, daemon=True)
        sub_thread.start()
        
        cv2.namedWindow("RF Heatmap", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("RF Heatmap", 700, 512)
        
        frame_count = 0
        last_img = None
        
        while self.running:
            # Get latest image
            try:
                img = self.image_queue.get(timeout=0.1)
                last_img = img
                frame_count += 1
            except queue.Empty:
                img = last_img
            
            if img is not None:
                # Scale up for visibility
                display = cv2.resize(img, (512, 512), interpolation=cv2.INTER_NEAREST)
                display = self.add_legend(display)
                
                # Add frame counter
                cv2.putText(display, f"Frame: {frame_count}", (10, 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                cv2.imshow("RF Heatmap", display)
            else:
                # Show waiting message
                waiting = np.zeros((512, 600, 3), dtype=np.uint8)
                cv2.putText(waiting, "Waiting for heatmap data...", (100, 250),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(waiting, f"Topic: {self.topic}", (150, 290),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                cv2.imshow("RF Heatmap", waiting)
            
            key = cv2.waitKey(50) & 0xFF
            if key == ord('q'):
                self.running = False
            elif key == ord('s') and last_img is not None:
                filename = f"heatmap_{frame_count:04d}.png"
                # Save full resolution
                cv2.imwrite(filename, last_img)
                print(f"Saved {filename}")
        
        # Cleanup
        if self.process:
            self.process.terminate()
        cv2.destroyAllWindows()


def signal_handler(sig, frame):
    print("\nShutting down...")
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    
    viewer = HeatmapViewer(topic="/gnb/heatmap", width=128, height=128)
    viewer.run()