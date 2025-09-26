#!/usr/bin/env python3
import cv2
import time

print("Testing video device...")
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open video device 0")
    exit(1)

print("Video device opened successfully")
print("Reading frames...")

for i in range(10):
    ret, frame = cap.read()
    if ret:
        print(f"Frame {i}: shape={frame.shape}, type={frame.dtype}")
    else:
        print(f"Frame {i}: failed to read")
    time.sleep(0.1)

cap.release()
print("Test completed")




