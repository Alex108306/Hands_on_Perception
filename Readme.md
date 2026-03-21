# Hands-on Perception Labs

This repository contains code and resources for two perception labs:

## Lab 1: ArUco Marker Detection & Augmented Reality

- **Objective:**  
    Learn to detect ArUco markers in images and videos, generate custom markers, and apply simple augmented reality overlays.
- **Key Features:**
    - Generate and print ArUco markers.
    - Detect markers in real-time video streams.
    - Estimate marker pose and overlay virtual objects for AR effects.

## Lab 2: Event-Based Camera Processing

- **Objective:**  
    Explore event-based camera data processing using two visualization methods: accumulation and Gaussian filtering.
- **Key Features:**
    - Process event streams from an event-based camera.
    - Visualize events using:
        - **Accumulate Method:** Integrate events over time to form an image.
        - **Gaussian Method:** Apply Gaussian filtering for smoother event visualization.
    - Includes sample video demonstrations of both methods.

## Usage

1. Clone the repository:
     ```bash
     git clone <repo-url>
     cd Hands_on_Perception
     ```

2. Follow instructions in each lab's folder for setup and running the code.

## Video Demonstrations

- Lab 2 includes video files showing the output of both the accumulate and Gaussian methods.  
    Check the `lab2/data/` directory for sample results.

[![Accumulate Method Output]](lab_2/lab/data/indoor_flying4_event_stream.mp4)
[![Gaussian Method Output]](lab_2/lab/data/indoor_flying4_images.mp4)

---

For detailed instructions, see the respective lab folders.
