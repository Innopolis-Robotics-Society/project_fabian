# **Project Fabian - "Robodog-human interaction with CV and deep learning"**

ROS 2 stack for a Unitree A1 quadruped with an onboard Jetson Orin Nano providing:
- real-time human and pose detection (YOLO-pose, ONNX Runtime);
- gesture recognition from skeletons (ST-GCN);
- high-level locomotion commands for A1;
- the ability to run on the real robot or locally on a GPU PC for neural network debugging.

## 1. Overview and architecture

### 1.1. Dataflow

1. An RGB camera (on Jetson / local machine) publishes a video stream.
2. `f_human_detection2`:
   - runs pose detection model (YOLO-pose in ONNX format);
   - detects people and skeletal keypoints;
   - publishes `PersonBodyArray` messages for downstream modules;
   - optionally publishes a debug image and RViz configuration.
3. `f_gesture_recognition`:
   - accumulates temporal sequences of skeletons;
   - classifies gestures using ST-GCN;
   - publishes `PersonAction` messages.
4. `f_fox_command`:
   - converts gestures and scene state into motion commands;
   - generates `FoxCommand` messages (velocity, turn rate, modes, etc.).
5. `f_a1_lcm_control`:
   - receives `FoxCommand`;
   - converts it to Unitree SDK format (via LCM + `unitree_legged_sdk`);
   - sends low-level commands to A1.
6. `unitree_ros2` and `unitree_hardware` provide integration with the physical robot and simulation.

---

## 2. Repository structure

Main top-level directories:

- `f_human_detection2/` — human pose detection:
  - `f_human_detection2/pose_node.py` — main ROS 2 node;
  - `f_human_detection2/backends/` — inference backends:
    - `yolo11_pose_onnxrt.py` — ONNX Runtime + CUDA;
    - `yolo11_pose_trt.py` — experimental TensorRT backend ( disabled by default);
  - `config/dev.yaml`, `config/prod.yaml` — configs for local and onboard modes;
  - `launch/dev.launch.py`, `launch/prod.launch.py`, `launch/dummy.launch.py` — ready-to-use launch files;
  - `models/` — ONNX models and auxiliary files;
  - `rviz/pose_debug.rviz` — RViz config for debugging.

- `f_gesture_recognition/` — gesture recognition:
  - `pose_classifier.py` — ST-GCN inference on skeletons;
  - `models/` — ST-GCN models (pretrained and custom);
  - `examples/people.yaml` — example labels/config for gestures.

- `f_fox_command/` — high-level command logic:
  - `command_node.py` — ROS 2 node mapping gestures to A1 commands.

- `f_a1_lcm_control/` — LCM bridge to Unitree A1:
  - `a1_lcm_control_node.py` — bridge between ROS 2 messages and Unitree SDK.

- `f_bringup/` — launching the full stack:
  - `launch/bringup.launch.py` — perception + gesture + command + control;
  - `launch/tf.launch.py` — TF tree configuration.

- `f_interfaces/` — custom ROS 2 messages:
  - `msg/FoxCommand.msg` — robot commands;
  - `msg/PersonBody.msg`, `msg/PersonBodyArray.msg` — skeletons;
  - `msg/PersonAction.msg` — recognized actions/gestures.

- `ml_training/` — gesture model training:
  - `prepare_dataset.ipynb` — dataset preparation from skeletons;
  - `stgcn_custom.py` — ST-GCN fine-tuning.

- `unitree_legged_sdk/`, `unitree_ros2/` — external Unitree code:
  - SDK, drivers, and ROS 2 integration for A1.
  - Typically unchanged unless really needed.

- `util/` — utility scripts:
  - `animate_skeleton.py` — skeleton visualization;
  - `converse.py`, `lcm_setup.sh` — service scripts.

- `Dockerfile`, `DockerfileJetson` — base images for x86_64 development and Jetson builds.
- `docker-compose.yaml` — services for running the stack in Docker.
- `requirements-docker.txt` — Python dependencies for containers.

---

## 3. Hardware and software requirements

### 3.1. Hardware

Recommended setup:

- Unitree A1 quadruped (controlled via Unitree SDK).
- Onboard computer:
  - NVIDIA Jetson Orin Nano (or similar Jetson board) with CUDA-capable GPU;
  - or a desktop/server with NVIDIA GPU (for local debugging).
- RGB camera:
  - USB/CSI camera exposed as `/dev/videoX`, or the robot’s onboard camera.
- Developer host machine (Linux, ideally Ubuntu 20.04/22.04).

### 3.2. Software (host)

- Docker ≥ 20.10.
- Docker Compose (v2, `docker compose`).
- NVIDIA GPU driver + CUDA (on host, optional).
- NVIDIA Container Toolkit (`nvidia-container-toolkit`) for CUDA in Docker (optional).
- Git with submodule support.

For optional non-Docker runs:

- Python 3.10+;
- ROS 2 (matching the `package.xml` requirements);
- `colcon` + standard ROS 2 build tools;
- ONNX Runtime with CUDA support and (optionally) TensorRT Execution Provider.

---

## 4. Installation

### 4.1. Clone the repository

```bash
git clone --recurse-submodules git@github.com:Innopolis-Robotics-Society/project_fabian.git
cd project_fabian
```

If it was cloned earlier without submodules:

```bash
git submodule update --init --recursive
```

### 4.2. Prepare GPU Docker (once per host)

1. Install NVIDIA driver and CUDA for your OS/GPU.
2. Install NVIDIA Container Toolkit (see NVIDIA docs for details).
3. Configure Docker to use the NVIDIA runtime and restart the Docker daemon.

---

## 5. Quick start (Docker)

### 5.1. Local development (GPU PC, no robot)

1. Build and start the dev container:

   ```bash
   # TODO: adjust service name to match docker-compose.yaml
   docker compose up --build <service>
   ```

  > *note:* `terminal` service is for CPU running, `terminal` service is for CUDA running

2. Attach another shell to the running container:

   ```bash
   docker compose exec <service> bash
   ```

3. Inside the container (once after code/dependency changes):

   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

4. Run detection/gesture stack with a webcam:

   ```bash
   # TODO: adjust launch arguments as needed
   ros2 launch f_human_detection2 dev.launch.py

   # in another terminal inside the container:
   ros2 run f_gesture_recognition pose_classifier
   ```

5. For model experiments, use `ml_training/`:

   * open `prepare_dataset.ipynb` in Jupyter;
   * run `stgcn_custom.py` for fine-tuning.

### 5.2. Running on the robot (Jetson + A1)

1. Build and start the dev container:

   ```bash
   # TODO: adjust service name to match docker-compose.yaml
   docker compose up --build fox
   ```
2. Attach another shell to the running container:

   ```bash
   docker compose exec fox bash
   ```

3. Inside the container:

   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

4. Start Unitree low-level components (ROS 2 + SDK):

   ```bash
   # TODO: fill with actual commands from unitree_ros2/unitree_hardware
   ```
   
---

## 6. Configuration

Main configs:

* `f_human_detection2/config/dev.yaml` — local mode:

  * camera selection (`/dev/videoX` or ROS image topic);
  * input frame size, FPS;
  * detection thresholds, NMS, etc.
* `f_human_detection2/config/prod.yaml` — onboard mode:

  * robot camera parameters;
  * tuned thresholds for real-world usage.
* `f_gesture_recognition`:

  * list of supported gestures;
  * temporal window length (number of frames);
  * path to ST-GCN model.
* `f_fox_command`:

  * gesture → command mapping;
  * velocity/turn limits;
  * safety parameters.

> TODO: add more.

---

## 8. Roadmap (example)

Short-term:

* [ ] Finalize the Jetson Docker image (smaller, more robust runtime).
* [ ] Document all launch files and typical run scenarios.

Mid-term:

* [ ] Enable more optimized inference backends (FP16 / INT8, optionally TensorRT).
* [ ] Extend the gesture set and support multiple people in frame.
* [ ] Improve HRI scenarios (approach, follow, safe stop, etc.).

Long-term:

* [ ] Support additional platforms (Go1, Aliengo, etc.).
* [ ] Integrate with trajectory planning and mapping systems.
