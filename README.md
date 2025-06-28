# 🚁 Drone Safe Landing Spot Detection & Landing Simulation

This project performs automated detection of safe landing zones for drones using a static aerial image. It uses computer vision techniques to identify flat and bright regions in the image, filters out unsafe areas, and simulates the drone flying and landing at the nearest suitable spot.

---

## 📌 Features

- ✅ Detects safe drone landing zones from an aerial image.
- ✅ Combines flatness and brightness masks for terrain evaluation.
- ✅ Filters regions by area, shape, convexity, and uniformity.
- ✅ Estimates real-world sizes using adaptive meters-per-pixel (MPP).
- ✅ Simulates drone movement and descent toward the selected zone.
- ✅ Visualizes detection steps and drone flight path using OpenCV.

---

## 📂 Directory Structure

```text

├── edge_detection.py            # Previous script for safe spot detection
├── main.py                      # Main script for detection and simulation
├── test_case_1.jpg               # Sample aerial image (user-provided)
├── test_case_2.jpg               # Sample aerial image (user-provided)
├── test_case_3.jpg               # Sample aerial image (user-provided)
├── test_case_4.jpg               # Sample aerial image (user-provided)
├── test_case_5.jpg               # Sample aerial image (user-provided)
└── README.md                    # Project documentation
```

## ⚙️ Requirements
- Python 3.7 or above
- OpenCV
- NumPy

Install dependencies:
```text
pip install opencv-python numpy
```

## 🛠️ How to Use

### 1. Prepare your image
Replace the path in main.py with the path to your test aerial image:

``` bash
image_path = r"C:\MEET\IROC_ISRO\Codes\test_pic_2.jpg
```

### 2. Run the script
```text
python main.py
```

### What you get:
1. A window showing detected landing zones.
2. A simulation of the drone flying and landing.
3. Two output images:
    - landing_zones_output.jpg: highlights detected spots.
    - landing_simulation_final.jpg: shows final position after simulated landing.
  
## 🧠 How It Works
### Step 1: Preprocessing
- Image is resized and converted to LAB color space.
- Brightness channel is normalized and equalized using CLAHE.
- A gradient map is computed using Sobel filters.

### Step 2: Safe Zone Detection
- Flatness Mask: Low-gradient regions.
- Brightness Mask: Well-lit areas.
- Combined mask isolates flat and bright zones.
- Contours are filtered based on:
- Minimum size in meters²
- Convexity
- Brightness uniformity
- Aspect ratio

### Step 3: Nearest Spot Selection
- The region closest to the center is selected as the preferred landing zone.

### Step 4: Drone Movement Simulation
- Simulates drone movement frame by frame toward the target.
- Descent is visualized over time.
- Final frame is saved showing “Landed” status.


## 🛫 Future Enhancements (TODO)
- Real-time video stream input
- GPS coordinate mapping using localization
- Integration with actual MAVLink drones
- Add terrain elevation model support for better analysis

## 👨‍💻 Author
Developed for experimental drone vision simulation and IROC/ISRO integration tasks.
If you found this useful or have questions, feel free to fork the repo, open an issue, or contribute!

## 📄 License
This project is licensed under the MIT License.
Feel free to use, modify, and share — with credit where due.
