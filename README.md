# Utils

## Instructions
- Install evtest with
`sudo apt-get install evtest`
- Install ROS2 [Ubuntu 22.04: [humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) / Ubuntu 20.04: [foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)]
- Install ROS1 noetic
- Create stitcher conda environment with `conda env create -f stitcher.yaml`
- [TODO: Improve] place shahram's BEV code in `~/Desktop/perciv_svd/SVD_bev_stitched`
- Run `bash setup_computer.sh` to create virtual environment and ros workspace

- Wohoo: run `bash start_demo.sh` to launch the demo!
- Or: Modify the application with your username and put in ~/.local/share/applications/PERCIV_DEMO.desktop
- Shahram hack to solve all life problems: rm ${CONDA_PREFIX}/lib/libffi.7.so ${CONDA_PREFIX}/lib/libffi.so.7
