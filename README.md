<p align="center">
<img src="https://raw.githubusercontent.com/xdspacelab/openvslam/master/docs/img/logo.png" width="435px">
</p>

# OpenVSLAM: A Versatile Visual SLAM Framework
[![Wercker Status](https://app.wercker.com/status/8b02a43f48216385658bb3857aae5fd8/s/master)](https://app.wercker.com/shinsumicco/openvslam/runs)
[![Travis CI Status](https://api.travis-ci.org/xdspacelab/openvslam.svg?branch=master)](https://travis-ci.org/xdspacelab/openvslam)
[![Documentation Status](https://readthedocs.org/projects/openvslam/badge/?version=master)](https://openvslam.readthedocs.io/)
[![Docker Build Status](https://img.shields.io/docker/cloud/build/shinsumicco/openvslam.svg)](https://hub.docker.com/r/shinsumicco/openvslam)
[![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)

## Overview

[<img src="https://raw.githubusercontent.com/xdspacelab/openvslam/master/docs/img/teaser.png" width="640px">](https://arxiv.org/abs/1910.01122)

[<img src="https://raw.githubusercontent.com/wiki/xdspacelab/openvslam/media/tracking.gif" width="640px">](https://www.youtube.com/watch?v=Ro_s3Lbx5ms)

[**[PrePrint]**](https://arxiv.org/abs/1910.01122) [**[YouTube]**](https://www.youtube.com/watch?v=Ro_s3Lbx5ms)

OpenVSLAM is a monocular, stereo, and RGBD visual SLAM system.
The notable features are:

- It is compatible with **various type of camera models** and can be easily customized for other camera models.
- Created maps can be **stored and loaded**, then OpenVSLAM can **localize new images** based on the prebuilt maps.
- The system is fully modular. It is designed by encapsulating several functions in separated components with easy-to-understand APIs.
- We provided **some code snippets** to understand the core functionalities of this system.

OpenVSLAM is based on an indirect SLAM algorithm with sparse features, such as ORB-SLAM, ProSLAM, and UcoSLAM.
One of the noteworthy features of OpenVSLAM is that the system can deal with various type of camera models, such as perspective, fisheye, and equirectangular.
If needed, users can implement extra camera models (e.g. dual fisheye, catadioptric) with ease.
For example, visual SLAM algorithm using **equirectangular camera models** (e.g. RICOH THETA series, insta360 series, etc) is shown above.

Some code snippets to understand the core functionalities of the system are provided.
You can employ these snippets for in your own programs.
Please see the `*.cc` files in `./example` directory or check [Simple Tutorial](https://openvslam.readthedocs.io/en/master/simple_tutorial.html) and [Example](https://openvslam.readthedocs.io/en/master/example.html).

We provided [documentation](https://openvslam.readthedocs.io/) for installation and tutorial.
Please contact us via [GitHub issues](https://github.com/xdspacelab/openvslam/issues) if you have any questions or notice any bugs about the software.

## Motivation

Visual SLAM is regarded as a next-generation technology for supporting industries such as automotives, robotics, and xR. 
We released OpenVSLAM as an opensource project with the aim of collaborating with people around the world to accelerate the development of this field.
In return, we hope this project will bring safe and reliable technologies for a better society.

## Installation

Please see [**Installation**](https://openvslam.readthedocs.io/en/master/installation.html) chapter in the [documentation](https://openvslam.readthedocs.io/).

[**The instructions for Docker users**](https://openvslam.readthedocs.io/en/master/docker.html) are also provided.

## Tutorial

Please see [**Simple Tutorial**](https://openvslam.readthedocs.io/en/master/simple_tutorial.html) chapter in the [documentation](https://openvslam.readthedocs.io/).

A sample ORB vocabulary file can be downloaded from [here](https://drive.google.com/open?id=1wUPb328th8bUqhOk-i8xllt5mgRW4n84).
Sample datasets are also provided at [here](https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4).

If you would like to run visual SLAM with standard benchmarking datasets (e.g. KITTI Odometry dataset), please see [**SLAM with standard datasets**](https://openvslam.readthedocs.io/en/master/example.html#slam-with-standard-datasets) section in the [documentation](https://openvslam.readthedocs.io/).

## Community

If you want to join our Slack community, please fill out the application form from the following site(s):

- [http://bit.ly/openvslam](http://bit.ly/openvslam)
- [http://bit.ly/openvslam-jp](http://bit.ly/openvslam-jp) (日本語: in Japanese)

## Currently working on

- IMU integration
- Python bindings
- Implementation of extra camera models
- Refactoring

Feedbacks, feature requests, and contribution are welcome!

## License

**2-clause BSD license** (see [LICENSE](./LICENSE))

The following files are derived from third-party libraries.

- `./3rd/json` : [nlohmann/json \[v3.6.1\]](https://github.com/nlohmann/json) (MIT license)
- `./3rd/popl` : [badaix/popl \[v1.2.0\]](https://github.com/badaix/popl) (MIT license)
- `./3rd/spdlog` : [gabime/spdlog \[v1.3.1\]](https://github.com/gabime/spdlog) (MIT license)
- `./src/openvslam/solver/pnp_solver.cc` : part of [laurentkneip/opengv](https://github.com/laurentkneip/opengv) (3-clause BSD license)
- `./src/openvslam/feature/orb_extractor.cc` : part of [opencv/opencv](https://github.com/opencv/opencv) (3-clause BSD License)
- `./src/openvslam/feature/orb_point_pairs.h` : part of [opencv/opencv](https://github.com/opencv/opencv) (3-clause BSD License)
- `./viewer/public/js/lib/dat.gui.min.js` : [dataarts/dat.gui](https://github.com/dataarts/dat.gui) (Apache License 2.0)
- `./viewer/public/js/lib/protobuf.min.js` : [protobufjs/protobuf.js](https://github.com/protobufjs/protobuf.js) (3-clause BSD License)
- `./viewer/public/js/lib/stats.min.js` : [mrdoob/stats.js](https://github.com/mrdoob/stats.js) (MIT license)
- `./viewer/public/js/lib/three.min.js` : [mrdoob/three.js](https://github.com/mrdoob/three.js) (MIT license)

Please use `g2o` as the dynamic link library because `csparse_extension` module of `g2o` is LGPLv3+.

## Contributors

- Shinya Sumikura ([@shinsumicco](https://github.com/shinsumicco))
- Mikiya Shibuya ([@MikiyaShibuya](https://github.com/MikiyaShibuya))
- Ken Sakurada ([@kensakurada](https://github.com/kensakurada))

## Citation

OpenVSLAM **won first place** at **ACM Multimedia 2019 Open Source Software Competition**.

If OpenVSLAM helps your research, please cite the paper for OpenVSLAM. Here is a BibTeX entry:

```
@inproceedings{openvslam2019,
  author = {Sumikura, Shinya and Shibuya, Mikiya and Sakurada, Ken},
  title = {{OpenVSLAM: A Versatile Visual SLAM Framework}},
  booktitle = {Proceedings of the 27th ACM International Conference on Multimedia},
  series = {MM '19},
  year = {2019},
  isbn = {978-1-4503-6889-6},
  location = {Nice, France},
  pages = {2292--2295},
  numpages = {4},
  url = {http://doi.acm.org/10.1145/3343031.3350539},
  doi = {10.1145/3343031.3350539},
  acmid = {3350539},
  publisher = {ACM},
  address = {New York, NY, USA}
}
```

The preprint can be found [here](https://arxiv.org/abs/1910.01122).

## Reference

- Raúl Mur-Artal, J. M. M. Montiel, and Juan D. Tardós. 2015. ORB-SLAM: a Versatile and Accurate Monocular SLAM System. IEEE Transactions on Robotics 31, 5 (2015), 1147–1163.
- Raúl Mur-Artal and Juan D. Tardós. 2017. ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras. IEEE Transactions on Robotics 33, 5 (2017), 1255–1262.
- Dominik Schlegel, Mirco Colosi, and Giorgio Grisetti. 2018. ProSLAM: Graph SLAM from a Programmer’s Perspective. In Proceedings of IEEE International Conference on Robotics and Automation (ICRA). 1–9.
- Rafael Muñoz-Salinas and Rafael Medina Carnicer. 2019. UcoSLAM: Simultaneous Localization and Mapping by Fusion of KeyPoints and Squared Planar Markers. arXiv:1902.03729.
- Mapillary AB. 2019. OpenSfM. https://github.com/mapillary/OpenSfM.
- Giorgio Grisetti, Rainer Kümmerle, Cyrill Stachniss, and Wolfram Burgard. 2010. A Tutorial on Graph-Based SLAM. IEEE Transactions on Intelligent Transportation SystemsMagazine 2, 4 (2010), 31–43.
- Rainer Kümmerle, Giorgio Grisetti, Hauke Strasdat, Kurt Konolige, and Wolfram Burgard. 2011. g2o: A general framework for graph optimization. In Proceedings of IEEE International Conference on Robotics and Automation (ICRA). 3607–3613.


# Personal notes for running it using ROS2

It already contains the dependencies vision_opencv and image_common

## Dependencies

### ROS 2

I used [ROS2 dashing](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)


### Vision openCV and Image Common packages

1. Source **ros2 dashing** setup files:

```bh
    source $HOME/openvslam/install/setup.bash
```

2. Build the vision_opencv and image_common packages 

```bh
    cd $HOME/openvslam/ros2
    colcon build --packages-select cv_bridge camera_calibration_parsers image_geometry image_transport opencv_tests vision_opencv camera_info_manager
```

### Openvslam packages

3. Source **image_common** and **vision_opencv** setup files

```bh
    source $HOME/openvslam/install/setup.bash
```

4. Build openvslam packages

```
    cd $HOME
    git clone -b ros2 --single-branch https://github.com/mirellameelo/openvslam.git
    cd openvslam
    mkdir build && cd build
    cmake     -DBUILD_WITH_MARCH_NATIVE=ON     -DUSE_PANGOLIN_VIEWER=ON     -DUSE_SOCKET_PUBLISHER=OFF     -DUSE_STACK_TRACE_LOGGER=ON     -DBOW_FRAMEWORK=DBoW2     -DBUILD_TESTS=ON     ..
    make -j4
    sudo make install
    cd ../ros2
    colcon build --symlink-install
```

It should build more 2 packs:
- openvslam
- publisher

# Running with a dataset 

1. Download:

    a. video.mp4 file
    b. config.yaml file
    c. vocabulary.dbow2 file

```bh
    cd $HOME/openvslam/build
    
    # video and config files
    FILE_ID="1d8kADKWBptEqTF7jEVhKatBEdN7g0ikY"
    curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${FILE_ID}" > /dev/null
    CODE="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
    curl -sLb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${CODE}&id=${FILE_ID}" -o aist_living_lab_1.zip
    unzip aist_living_lab_1.zip
    
    # vocabulary
    FILE_ID="1wUPb328th8bUqhOk-i8xllt5mgRW4n84"
    curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${FILE_ID}" > /dev/null
    CODE="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
    curl -sLb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${CODE}&id=${FILE_ID}" -o orb_vocab.zip
    unzip orb_vocab.zip
```

2. Open 3 terminals and source **vision_opencv**, **image_common** and **openvslam** setup files in each terminal:

```bh
    source $HOME/openvslam/install/setup.bash
```

**Terminal 1**: publish the video
```bh
    ros2 run publisher video -m $HOME/openvslam/build/aist_living_lab_1/video.mp4
```

**Terminal 2**: republish the video
```bh
    ros2 run image_transport republish raw in:=/video/image_raw raw out:=/camera/image_raw
```

**Terminal 3**: run_slam mode
```bh
     ros2 run openvslam run_slam -v $HOME/openvslam/build/orb_vocab/orb_vocab.dbow2 -c $HOME/openvslam/build/aist_living_lab_1/config.yaml 
     # OR to save the map
     ros2 run openvslam run_slam -v $HOME/openvslam/build/orb_vocab/orb_vocab.dbow2 -c $HOME/openvslam/build/aist_living_lab_1/config.yaml --eval-log --map-db $HOME/openvslam/ros2/mymap.msg
```

# Running with a USB camera

1. Open 3 terminals and source **vision_opencv**, **image_common** and **openvslam** setup files in each terminal:

```bh
    source $HOME/openvslam/install/setup.bash
```

**Terminal 1**: publish the video
```bh
    ros2 run image_tools cam2image -t camera 
```

**Terminal 2**: republish the video
```bh
    ros2 run image_transport republish raw in:=/camera raw out:=/camera/image_raw
```

**Terminal 3**: run_slam mode or run_localization mode
```bh
     # SLAM MODE
     ros2 run openvslam run_slam -v $HOME/openvslam/build/orb_vocab/orb_vocab.dbow2 -c <CONFIG_PATH>/config.yaml 
     # OR to save the map
     ros2 run openvslam run_slam -v $HOME/openvslam/build/orb_vocab/orb_vocab.dbow2 -c <CONFIG_PATH>/config.yaml --eval-log --map-db $HOME/openvslam/ros2/map.msg

     # localization mode (once you alread have a map.msg)
     ros2 run openvslam run_localization -v  $HOME/openvslam/build/orb_vocab/orb_vocab.dbow2 -c <CONFIG_PATH>/config.yaml --map-db $HOME/openvslam/ros2/map.msg
```
