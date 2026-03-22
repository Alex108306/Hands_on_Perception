# Hands-on Perception Labs

This repository contains code and resources for two perception labs:

## Lab 1: ArUco Marker Detection & Augmented Reality

- **Objective:**  
    Learn to detect ArUco markers in images and videos, generate custom markers, and apply simple augmented reality overlays.
- **Key Features:**
    - Generate and print ArUco markers.
    - Detect markers in real-time video streams.
    - Estimate marker pose and overlay virtual objects for AR effects.
    - Estimate pose relative and distance between two aruco marker poses

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

Lab 2 includes video files showing the output of Gaussian methods.

- **Gray Scale Video:**  
  ![Accumulate Method Output](lab_2/lab/videos/indoor_flying4_images.gif)

- **Accumulate Method Output:**  
  ![Gaussian Method Output](lab_2/lab/videos/indoor_flying4_event_stream_accumulate_method.gif)

- **Gaussian Method Output:**  
  ![Gaussian Method Output](lab_2/lab/videos/indoor_flying4_event_stream_gaussian_method.gif)

*For higher quality or longer videos, see the MP4 files in the same directory.*

---

For detailed instructions, see the respective lab folders.
