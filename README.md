- In branch `main` you will find the ros2 structure
- In branch `cli_tool` you will find the cli_tool for the enhancement
# main
To launch the enhancer node:
```bash
ros2 launch image_pipeline_launcher vision_pipeline_launch.py
```
To launch the segmentation calibration node:
```bash
ros2 launch image_pipeline_launcher segmentation_calibrator.py
```
To start the publishing of the image of the pipe:
```bash
python python_img_publisher.py
```
Just remember to change the path of the file inside the script

# Ros2 install instruction
Clone this repo inside the src folder of your workspace

# NOTES
This repo must be built with `--symlink-install`

Tested with:
- Nvidia driver: `570.133.07`
- Cuda version: `12.8`
- tensorrt version: `10.10.0`
- Graphics card generation: `Tesla`
- gcc version `10`
- g++ version `10`
- Opencv version `4.10.0`
