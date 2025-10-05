# hike_grid

Adaptive Terrain VR Treadmill - inspired by life in mountains

# Project description

The **Adaptive Terrain VR Treadmill** is a **shape-shifting 360° VR locomotion platform** inspired by life in the mountains. Under each foot, a dense grid of fast-moving pins sculpts a true three-dimensional terrain — rocks, roots, steps — synchronized with the virtual world. The rotating base allows users to maintain natural orientation, while optional tilt and haptic effects add incline, wind, and vibration. We capture trail segments using **LiDAR or photogrammetry** and convert them into high-resolution elevation maps that control the pins in real time. The result is **realistic terrain**, safer training, and deeper immersion than flat multidirectional treadmills. Main applications: elite-level sports training, rehabilitation with programmable difficulty levels, and field training for rescuers or soldiers. The system is designed for low latency, high safety, and modular scalability, with the ability to attach vertical walls with pegs that mimic rock surfaces and enable **realistic climbing in VR**. Climb indoors, feel the ground beneath your feet, and bring authentic mountain movements to VR. Designed for demonstrations, pilot projects, and production readiness.

# Dependenciesd

- ROS2 (humble)

# What software we are using (main libraries)

- ROS2
- DepthAI
- OpenCV
- Open3d

# installation

1. Build workspace
2. source installation
3. run `ros2 launch depthai_ros_driver rtabmap.launch.py`
4. flash firmware to esp32 from scripts/*