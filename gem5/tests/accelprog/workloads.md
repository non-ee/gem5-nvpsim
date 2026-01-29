# HAR
## 1. Application Overview
This workload implements a step detection algorithm for Human Activity Recognition (HAR), simulating wearable fitness tracker functionality. It processes 3-axis accelerometer data to identify walking steps in real-time, representing a common embedded IoT application with well-defined sensing, processing, and communication phases. The workload captures the essential characteristics of battery-powered wearable devices that must balance computational accuracy with energy efficiency.

## 2. Input / Output
**Input Specifications**:
- Data Type: 3-axis accelerometer readings (X, Y, Z)
- Format: 16-bit signed integers scaled by 1000
- Sampling: 10 Hz for 2 seconds (20 samples total)
- Memory: 120 bytes input buffer (20×3×2 bytes)
- Content: Synthetic walking pattern with gravity bias and sensor noise

**Output Specifications**
- Primary: 32-bit step count (4 bytes)
- Secondary: Binary step detection array (20 bytes, 1=step, 0=no step)
- Transmission: Step count transmitted via RF interface (4 bytes)
- Display: Step statistics and detection pattern

## 3. Algorithm Flows
**Processing Pipeline**
```text
Accelerometer Data → Gravity Removal → Magnitude Calculation → 
Moving Average Filter → Threshold Detection → Step Counting → RF Transmission
```

Key Computational Stages:
- Gravity Compensation: Removes DC bias from vertical axis using first sample as baseline
- Signal Magnitude Calculation: Computes vector magnitude √(x²+y²+z²) for each sample
- Moving Average Filtering: Applies 5-sample window to smooth magnitude data
- Threshold-Based Detection: Identifies steps when magnitude exceeds both relative (1.5× average) and absolute (1.2g) thresholds
- Debouncing Logic: Prevents multiple detections from single steps by requiring minimum time separation
- Result Aggregation: Counts valid detections and calculates step frequency

# Image Processing**
## 1. Application Overview
This workload implements basic image filtering operations simulating embedded vision systems such as surveillance cameras or industrial inspection devices. It performs convolution-based operations on grayscale image data, representing computational-intensive applications that benefit from hardware acceleration. The workload models edge detection and noise reduction algorithms common in computer vision.
The architecture follows the "pixels-to-insight" model proposed by Sampson et al [], which reconfigures the traditional imaging pipeline to prioritize efficient feature extraction over human-centric image quality. This approach provides a representative trace for evaluating state retention and peripheral consistency in intermittent computing systems.

## 2. Input / Output
**Input Specifications**
- Data Format: 8-bit grayscale pixels (0-255 range) stored in raw_image[] buffer
- Resolution: Fixed 8×8 pixels (64 bytes total) - intentionally small for embedded constraints
- Capture Method: Pixel-by-pixel via camCapturePixel() simulating a virtual camera device
- Memory Layout: Row-major order in IMAGE_SIZE (8×8 = 64) element array

**Output Specifications**
- Primary Output: Edge-detected binary image stored in edge_map[] buffer
- Feature Detection: Counts stored in feature_count, motion in motion_level
- Compressed Output: Run-length encoded edge map (variable size, typically < 32 bytes)
- Transmission Format: 4-byte header + compressed data via rfTransmitByte()

## 3. Algorithm Flows
**Processing Pipeline**
```text
capture_image() → apply_filter(GAUSSIAN) → apply_filter(EDGE) → 
detect_features() → compress_results() → transmit_data()
```

**Key Computational Stages**
- Image Capture
A virtual camera device generates synthetic 8×8 test patterns with realistic variations while automatically detecting motion between consecutive frames.
- Noise Reduction
A 3×3 Gaussian blur filter smoothes the image to remove sensor noise and small artifacts, preparing it for accurate edge detection.
- Edge Detection
Sobel operators calculate image gradients and apply thresholding to produce a binary edge map highlighting significant boundaries and shapes.
- Feature Extraction
The system counts detected edges and corners while quantifying motion levels to determine the dominant visual features present.
- Data Compression
Run-length encoding efficiently compresses the binary edge map, typically reducing data size by 50-75% for transmission.
- Data Transmission
A 4-byte metadata header and compressed edge data transmit wirelessly using the same reliable protocol as other sensor systems.


# Wildlife Audio Classifier
## 1. Application Overview
This workload implements a simple wildlife audio classifier for environmental monitoring applications. It captures audio samples, analyzes frequency characteristics, and classifies detected sounds into different animal categories (birds, mammals, insects). The workload models bioacoustic monitoring devices used in ecological research, wildlife conservation, and smart agriculture systems, where power efficiency and real-time classification are essential.

## 2. Input / Output
**Input Specifications**
- Data Type: 16-bit audio amplitude samples
- Sampling Rate: 20 Hz (50ms interval between samples)
- Duration: 1 second (20 samples total)
- Memory: 40 bytes input buffer (20×2 bytes)
- Format: Raw microphone readings with ambient noise

**Output Specifications**
- Primary: Animal classification result (bird, mammal, insect, none, unknown)
- Secondary: Confidence score (0-100%) and detected frequency
- Transmission: 3-byte packet [animal_type][confidence][checksum]
- Display: Classification result with confidence percentage

## 3. Algorithm Flows
**Processing Pipeline**
```text
Audio Samples → Zero-Crossing Analysis → Frequency Estimation → 
Frequency-Based Classification → Confidence Scoring → RF Transmission
```

**Key Computational Stages**
Zero-Crossing Detection: Calculates signal crossings about the mean to estimate fundamental frequency
Frequency Calculation: Derives frequency from zero-crossing count and sampling duration
Range-Based Classification: Maps detected frequency to predefined animal vocalization ranges
Confidence Computation: Calculates certainty based on proximity to range centers
Result Packaging: Formats classification with checksum for transmission
