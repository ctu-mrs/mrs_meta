# Point-LIO

This is a fork of the original [`hku-mars/Point-LIO`](https://github.com/hku-mars/Point-LIO) implementation and as such follows its licensing.
If you use this implementation in your work, please cite the original authors.

Our `ctu-mrs` implementation includes the following changes:
  * Major ROS2 refactoring
  * Update of the main loop
  * Integration with the MRS UAV System.

## About

<div align="center">
    <div align="center">
        <img src="https://github.com/hku-mars/Point-LIO/raw/master/image/toc4.png" width = 75% >
    </div>
    <font color=#a0a0a0 size=2>The framework and key points of the Point-LIO.</font>
</div>

**Point-LIO** is a robust and high-bandwidth LiDAR-inertial odometry with the capability to estimate extremely aggressive robotic motions. Point-LIO has two key novelties that enable a high-bandwidth LiDAR-inertial odometry (LIO). The first one is a point-by-point LIO framework, where the state is updated at each LiDAR point measurement without accumulating them into a frame. This point-by-point update allows an extremely high-frequency odometry output, significantly increases the odometry bandwidth, and also fundamentally removes the artificial in-frame motion distortion in aggressive motions. The second main novelty is a stochastic process-augmented kinematic model which models the IMU measurements as an output, instead of input as in existing filter-based odometry or SLAM systems, of the model. This new modeling method enables accurate localization and reliable mapping for aggressive motions even when IMU measurements are saturated. In experiments, Point-LIO is able to provide accurate, high-frequency odometry (4-8 kHz) and reliable mapping under severe vibrations and aggressive motions with high angular velocity (75 rad s^{-1}) beyond the IMU measuring ranges. And Point-LIO is computationally efficient, robust, versatile on public datasets with general motions. As an odometry, Point-LIO could be used in various autonomous tasks, such as trajectory planning, control, and perception, especially in cases involving very fast ego-motions (e.g., in the presence of severe vibration and high angular or linear velocity) or requiring high-rate odometry output and mapping (e.g., for high-rate feedback control and perception).

## Installation and launching

We recommend to install and launch Point-LIO from the [`mrs-ctu/mrs_point_lio_core`](https://github.com/ctu-mrs/mrs_point_lio_core/tree/ros2) repository.
Follow the Readme.

## Compatibility

The following descriptions have been removed from this Readme. Please see the original repository on how to use the features.
  * 
  * How to use Livox Avia, LiDARs with external IMU, Ouster/Velodyne.
  * Saving maps to PCD files.
  * Rosbag examples.

## FAQ

  * Make sure the IMU and LiDAR are **Synchronized**, that's important.
  * Obtain the saturation values of your used IMU (i.e., accelerator and gyroscope), and the units of the accelerator of your used IMU, then modify the `.yaml` file according to those settings, including values of `satu_acc`, `satu_gyro`, and `acc_norm`. That's improtant.
  * The warning message "Failed to find match for field 'time'." means that the timestamps of each LiDAR point are missing in the data. That is important since Point-LIO processes at the sampling time of each LiDAR point.
  * We recommend to set the `extrinsic_est_en = false` if the extrinsic is given. As for the extrinsic initialization, please refer to: [**Robust and Online LiDAR-inertial Initialization**](https://github.com/hku-mars/LiDAR_IMU_Init).
  * If you want to use Point-LIO without IMU, set the `imu_en = false` and provide a predefined value of gravity in `gravity_init` as true as possible in the `.yaml` file and keep the `use_imu_as_input = 0`.
