
# ROSbag Vision Pipeline

Offline processing pipeline for ROSbag files with object detection, geolocation, and image stitching.

## Quick Start

### 1. Setup
```bash
# Install dependencies
pip install opencv-python numpy pyyaml

# Build the package
cd ~/bv_ws
colcon build --packages-select bv_core
source install/setup.bash
```

### 2. Configure
Edit `rosbag_pipeline/config/pipeline_config.yaml` with your camera parameters:
```yaml
filtering:
  c_matrix: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
  dist_coefficients: [k1, k2, p1, p2, k3]
```

### 3. Run
```bash
# Terminal 1: Start processor
ros2 run bv_core bag_vision_processor \
  --config pipeline_config.yaml \
  --output ./output \
  --bag /path/to/bag

# Terminal 2: Play rosbag
ros2 bag play /path/to/your_bag.db3
```

### 4. Results
```
output/
├── annotated_frames/          # Frames with detection boxes
├── detections/
│   ├── detections.json       # All detections
│   └── geolocated_detections.csv
├── stitched/                 # Panoramas
└── processing_stats.json
```

## Features

- 📹 Extract images from rosbag files
- 🎯 Object detection (plug in RF-DETR, YOLO, etc.)
- 🌍 GPS geolocation of detections
- 🗺️ Automatic image stitching
- 📊 Export to JSON, CSV, GeoJSON

## Configuration

**Camera Calibration** - Get from your camera_info topic:
```bash
ros2 topic echo /camera_info
```

**Detection Model** - Add your model in `run_detection()` method:
```python
def run_detection(self, image, timestamp, frame_id):
    # Replace with your model
    results = your_model.predict(image)
    return detections
```

## Topics

Default topics (edit in config.yaml):
- `/image_raw` - Camera images
- `/mavros/global_position/global` - GPS
- `/mavros/global_position/rel_alt` - Altitude
- `/mavros/local_position/pose` - Pose

## Troubleshooting

**No detections found**
- Lower `detection_threshold` in config
- Check images are being received: `ros2 topic echo /image_raw`

**Geolocation incorrect**
- Verify camera orientation in config
- Check GPS data exists in bag: `ros2 bag info your_bag.db3`

**Out of memory**
- Reduce `batch_size` in config
- Set `save_raw_frames: false`

## Example: Full Pipeline

```bash
# 1. Record a flight
ros2 bag record /image_raw /mavros/global_position/global \
  /mavros/global_position/rel_alt /mavros/local_position/pose

# 2. Process the bag
ros2 run bv_core bag_vision_processor \
  --config pipeline_config.yaml \
  --output flight_001_results \
  --bag flight_001.
