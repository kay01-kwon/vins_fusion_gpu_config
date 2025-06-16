# vins_fusion_gpu_config

# Calibration cam - imu through Kalibr

Record the camera and imu data.

Then, run the Kalibr.

```
rosrun kalibr kalibr_calibrate_imu_camera --imu-models calibrated --reprojection-sigma 1.0 --target april.yaml -- imu imu.yaml --cams static_cam-camchain.yaml --bag dynamic_cam_imu.bag --show-extraction
```

## How to execute vins fusion with this config

```
rosrun vins vins_node vins_config/gazebo_sim/gazebo_sim.yaml
```

