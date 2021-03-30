# Map-Based Lidar-Camera Calibration
This app calibrates the lidar-camera pair by a novel map-based
calibration algorithm. The algorithm requires the following inputs:
* An initial calibration.
* An accurate lidar trajectory.
* A set of annotated 3d-2d point-to-point or 3d-2d line-to-line
  correspondecs, where the 3d points or 3d lines are selected from the
  hd map that is manually created from the accurate point cloud induced
  by the lidar trajectory.
  
The algorithm minimizes a 3D distance bewtween the annotationed 3d
vertex and its corresponding unprojected 2d vertex (unprojected using
ground truth 3d vertex's support plane).

The algorithm then outputs an optimized relative pose between the lidar
and the camera.

## Usage
To run the calibration program:
```
calibrate_lidar_camera \ 
    --seq_dir test_data 
    --correspondence_json_path test_data/3d_2d_correspondences_new.json 
```

## File locations and formats
* See `test_data/3d_2d_correspondences_new.json` for an example of the input 3d-2d
  correspondences file.
* See `test_data/optimized_lidar_to_camera_transform.json` for an
  example output calibration file.
