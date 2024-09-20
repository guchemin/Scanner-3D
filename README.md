# 3D Scanner System for Volume Calculation

## Overview

This repository contains the development of a 3D Scanner system aimed at determining the most effective methodology for calculating the volume of objects. The system uses an infrared sensor to capture distances between the sensor and the object, generating a point cloud that represents the object's surface. The proposed volume calculation methodologies are based on algorithms and geometric computations derived from the collected data.

## Features

- **Data Acquisition**: Uses an infrared sensor to collect distance measurements.
- **Point Cloud Generation**: Converts sensor data into a 3D point cloud representing the object's surface.
- **Volume Calculation**: Implements multiple algorithms to calculate object volumes based on the point cloud data.
- **Experimental Validation**: Provides experimental results showcasing the efficiency and accuracy of the system across various scenarios.

## System Overview

1. **Hardware**:
   - Infrared distance sensor for data acquisition.
   - Rotating platform to capture measurements at multiple angles.
   - Microcontroller to control sensor and platform movements.
   
2. **Software**:
   - Point cloud processing to convert raw sensor data into a 3D model.
   - Geometric algorithms for volume calculation, including methods based on approximations by areas and truncated cones or voxelization techniques.
   
## Methodologies

The system explores different approaches to calculate object volumes:
- **Approximation by Areas**: Based on the surface areas obtained from the point cloud.
- **Truncated Cones**: Uses the polar coordinates at various heights to calculate volumes via stacked truncated cones.
- **Voxelization**: Converts the point cloud into small 3D cubes (voxels) and calculates the volume based on the voxel grid.

## Implementation Stages

1. **Data Acquisition**: Capturing distance measurements at different angles and heights.
2. **Point Cloud Processing**: Generating a 3D point cloud from the acquired data.
3. **Volume Calculation**: Applying the chosen geometric algorithm to estimate the volume of the object.
4. **Experimental Validation**: Testing the system under various scenarios to measure its accuracy and limitations.

## Usage

To use this system:

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/3d-scanner-system.git
