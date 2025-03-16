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
