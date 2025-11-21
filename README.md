# mrs_meta

[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/ctu-mrs/mrs_meta)

A meta repository of ROS2 packages for data analysis, consisted mostly of packages mentioned in the [mrs_uav_system](https://github.com/ctu-mrs/mrs_uav_system/tree/ros2) repo and submodules from the following metapackages:

| Repository                                                                                        |
|---------------------------------------------------------------------------------------------------|
| [mrs_octomap_mapping_planning](https://github.com/ctu-mrs/mrs_octomap_mapping_planning/tree/ros2) |
| [mrs_open_vins_core](https://github.com/ctu-mrs/mrs_open_vins_core/tree/ros2)                     |
| [mrs_point_lio_core](https://github.com/ctu-mrs/mrs_point_lio_core/tree/ros2)                     |
| [mrs_uav_core](https://github.com/ctu-mrs/mrs_uav_core/tree/ros2)                                 |
| [mrs_uav_modules](https://github.com/ctu-mrs/mrs_uav_modules/tree/ros2)                           |

## Instructions

To update the subpackages remove every folder, make sure you have [gitman](https://gitman.readthedocs.io/en/latest/) and run:

```bash
gitman install
```
