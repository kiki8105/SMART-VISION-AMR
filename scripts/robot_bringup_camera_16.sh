#!/usr/bin/env bash
# Copy to the robot and run there. Starts no PC navigation or Gazebo processes.
set -eo pipefail

WORKSPACE=${TURTLEBOT3_WORKSPACE:-$HOME/turtlebot3_ws}
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
OPENCR_DEVICE=${OPENCR_DEVICE:-/dev/ttyACM0}
# The rear webcam's by-id link remains stable when its /dev/videoN number changes.
# Never substitute another camera when this device is disconnected.
DEFAULT_REAR_CAMERA_DEVICE=/dev/v4l/by-id/usb-BY-5640-200408_Webcam_01.00.00-video-index0
CAMERA_DEVICE=${CAMERA_DEVICE:-$DEFAULT_REAR_CAMERA_DEVICE}
ENABLE_FRONT_CAMERA=${ENABLE_FRONT_CAMERA:-false}
REAR_CAMERA_SIZE=${REAR_CAMERA_SIZE:-320x240}
FRONT_CAMERA_SIZE=${FRONT_CAMERA_SIZE:-320x240}
CAMERA_JPEG_QUALITY=${CAMERA_JPEG_QUALITY:-60}
CAMERA_PUBLISH_FPS=${CAMERA_PUBLISH_FPS:-5}
LOCALIZATION_MODE=${TURTLEBOT3_LOCALIZATION_MODE:-ekf_independent}
ESTIMATION_LOCATION=${SMART_VISION_ESTIMATION_LOCATION:-robot}
RUNTIME_WORKSPACE=${SMART_VISION_RUNTIME_WORKSPACE:-$HOME/smart_vision_runtime_ws}
LEGACY_EKF_LAUNCH=${SMART_VISION_ROBOT_EKF_LAUNCH:-$SCRIPT_DIR/hardware_robot_ekf.launch.py}
if [[ ! -f "$LEGACY_EKF_LAUNCH" && -f "$SCRIPT_DIR/../ros2_ws/src/smart_vision_amr/launch/hardware_robot_ekf.launch.py" ]]; then
  LEGACY_EKF_LAUNCH=$SCRIPT_DIR/../ros2_ws/src/smart_vision_amr/launch/hardware_robot_ekf.launch.py
fi
case "$ESTIMATION_LOCATION" in
  pc|robot) ;;
  *) printf 'SMART_VISION_ESTIMATION_LOCATION must be pc or robot.\n' >&2; exit 2 ;;
esac
case "$LOCALIZATION_MODE" in
  baseline|ekf_independent) ;;
  *) printf 'TURTLEBOT3_LOCALIZATION_MODE must be baseline or ekf_independent.\n' >&2; exit 2 ;;
esac
if [[ "$LOCALIZATION_MODE" == ekf_independent ]]; then
  CAMERA_BACKEND=${CAMERA_BACKEND:-rate_limited}
else
  CAMERA_BACKEND=${CAMERA_BACKEND:-v4l2}
fi
if [[ "$ESTIMATION_LOCATION" == pc && "$LOCALIZATION_MODE" == ekf_independent
      && ! -f "$LEGACY_EKF_LAUNCH" ]]; then
  printf 'Independent EKF bringup is missing: %s\n' "$LEGACY_EKF_LAUNCH" >&2
  printf 'Copy the launch file beside this script before starting the robot.\n' >&2
  exit 2
fi
case "$CAMERA_BACKEND" in
  rate_limited|v4l2) ;;
  *) printf 'CAMERA_BACKEND must be rate_limited or v4l2.\n' >&2; exit 2 ;;
esac
if ! [[ "$CAMERA_PUBLISH_FPS" =~ ^[1-9][0-9]*$ ]]; then
  printf 'CAMERA_PUBLISH_FPS must be a positive integer.\n' >&2
  exit 2
fi
if ! [[ "$CAMERA_JPEG_QUALITY" =~ ^[0-9]+$ ]] || (( CAMERA_JPEG_QUALITY < 1 || CAMERA_JPEG_QUALITY > 100 )); then
  printf 'CAMERA_JPEG_QUALITY must be an integer from 1 to 100.\n' >&2
  exit 2
fi
case "$ENABLE_FRONT_CAMERA" in
  true|false) ;;
  *) printf 'ENABLE_FRONT_CAMERA must be true or false.\n' >&2; exit 2 ;;
esac
case "$REAR_CAMERA_SIZE" in
  640x480|320x240) IFS=x read -r rear_width rear_height <<< "$REAR_CAMERA_SIZE" ;;
  *) printf 'REAR_CAMERA_SIZE must be 640x480 or 320x240.\n' >&2; exit 2 ;;
esac
case "$FRONT_CAMERA_SIZE" in
  640x480|320x240) ;;
  *) printf 'FRONT_CAMERA_SIZE must be 640x480 or 320x240.\n' >&2; exit 2 ;;
esac
if [[ ! -e "$CAMERA_DEVICE" ]]; then
  printf 'Camera device is missing: %s\n' "$CAMERA_DEVICE" >&2
  printf 'Connect the rear webcam, or set CAMERA_DEVICE explicitly for a different device.\n' >&2
  exit 2
fi

if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$WORKSPACE/install/setup.bash" ]]; then
  printf 'ROS or robot workspace setup is missing: %s/install/setup.bash\n' "$WORKSPACE" >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
if [[ "$ESTIMATION_LOCATION" == robot ]]; then
  if [[ ! -f "$RUNTIME_WORKSPACE/install/local_setup.bash" ]]; then
    printf 'Robot-local runtime is not built. Run scripts/setup_robot_runtime.sh on the robot first.\n' >&2
    printf 'Previous deployment: SMART_VISION_ESTIMATION_LOCATION=pc (set it on robot AND PC).\n' >&2
    exit 2
  fi
  source "$RUNTIME_WORKSPACE/install/local_setup.bash"
  if ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
    printf 'Robot runtime dependency missing: run scripts/setup_robot_runtime.sh on the robot.\n' >&2
    exit 2
  fi
  if ! RUNTIME_PREFIX=$(ros2 pkg prefix smart_vision_amr); then
    printf 'Robot runtime package missing: rebuild with scripts/setup_robot_runtime.sh.\n' >&2
    exit 2
  fi
  if [[ ! -f "$RUNTIME_PREFIX/share/smart_vision_amr/launch/hardware_robot_runtime.launch.py"
        || ! -x "$RUNTIME_PREFIX/lib/smart_vision_amr/motion_mux"
        || ! -x "$RUNTIME_PREFIX/lib/smart_vision_amr/localization_health" ]]; then
    printf 'Robot runtime overlay is outdated: rerun scripts/setup_robot_runtime.sh.\n' >&2
    exit 2
  fi
fi
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0 TURTLEBOT3_MODEL=waffle_pi
export LDS_MODEL=${LDS_MODEL:-LDS-03}
if [[ -f "$SCRIPT_DIR/dds_env.bash" ]]; then
  source "$SCRIPT_DIR/dds_env.bash" ""
elif [[ "${SMART_VISION_DDS_TRANSPORT:-udp}" == udp ]]; then
  printf 'Copy scripts/dds_env.bash beside this wrapper before enabling UDP transport.\n' >&2
  exit 2
fi

command -v setsid >/dev/null
ros2 pkg prefix turtlebot3_bringup >/dev/null
if [[ "$CAMERA_BACKEND" == rate_limited ]]; then
  if [[ ! -f "$SCRIPT_DIR/ros_camera_compressed.py" ]]; then
    printf 'Rate-limited camera node is missing: %s/ros_camera_compressed.py\n' "$SCRIPT_DIR" >&2
    exit 2
  fi
  python3 -c 'import cv2, rclpy, sensor_msgs.msg' >/dev/null
else
  if ! ros2 pkg prefix v4l2_camera >/dev/null 2>&1; then
    printf 'Install the camera driver on this robot: sudo apt install ros-humble-v4l2-camera\n' >&2
    exit 2
  fi
  if ! ros2 pkg prefix compressed_image_transport >/dev/null 2>&1; then
    printf 'Install the compressed image transport on this robot: sudo apt install ros-humble-compressed-image-transport\n' >&2
    exit 2
  fi
fi
if [[ ! -r "$CAMERA_DEVICE" || ! -w "$CAMERA_DEVICE" ]]; then
  printf 'Camera device is missing or inaccessible: %s\n' "$CAMERA_DEVICE" >&2
  exit 2
fi

# The CSI camera's /dev/videoN number can change when the rear USB camera is
# reconnected. Find the capture node named camera0 on each boot instead.
FRONT_CAMERA_DEVICE=${FRONT_CAMERA_DEVICE:-}
if [[ -z "$FRONT_CAMERA_DEVICE" ]]; then
  for class_device in /sys/class/video4linux/video*; do
    [[ -e "$class_device/name" ]] || continue
    if [[ $(<"$class_device/name") == camera0 ]]; then
      FRONT_CAMERA_DEVICE=/dev/"${class_device##*/}"
      break
    fi
  done
fi
front_camera_enabled=false
front_size=()
if [[ "$ENABLE_FRONT_CAMERA" == true && -n "$FRONT_CAMERA_DEVICE"
      && -r "$FRONT_CAMERA_DEVICE" && -w "$FRONT_CAMERA_DEVICE"
      && $(readlink -f "$FRONT_CAMERA_DEVICE") != $(readlink -f "$CAMERA_DEVICE") ]]; then
  if command -v v4l2-ctl >/dev/null; then
    front_formats=$(v4l2-ctl -d "$FRONT_CAMERA_DEVICE" --list-formats-ext 2>/dev/null || true)
    # v4l2_camera's verified YUYV -> rgb8 path is used only when this device
    # actually lists a practical capture size. Unsupported Bayer-only devices
    # are left untouched; rear bringup still works.
    front_selection=$(FRONT_FORMATS="$front_formats" FRONT_CAMERA_SIZE="$FRONT_CAMERA_SIZE" python3 - <<'PY'
import os
import re
formats, stepwise, current = {}, {}, None
for line in os.environ.get('FRONT_FORMATS', '').splitlines():
    entry = re.search(r"\[\d+\]: '([^']+)'", line)
    if entry:
        current = entry.group(1)
        formats.setdefault(current, set())
    size = re.search(r'Size: Discrete (\d+)x(\d+)', line)
    if size and current:
        formats[current].add((int(size.group(1)), int(size.group(2))))
    variable = re.search(
        r'Size: Stepwise (\d+)x(\d+) - (\d+)x(\d+) with step (\d+)/(\d+)', line)
    if variable and current:
        stepwise.setdefault(current, []).append(tuple(map(int, variable.groups())))
preferred = tuple(map(int, os.environ['FRONT_CAMERA_SIZE'].split('x')))
choices = list(dict.fromkeys((preferred, (640, 480), (320, 240))))
for width, height in choices:
    supported = (width, height) in formats.get('YUYV', set())
    for min_w, min_h, max_w, max_h, step_w, step_h in stepwise.get('YUYV', []):
        supported |= (step_w > 0 and step_h > 0 and min_w <= width <= max_w
                      and min_h <= height <= max_h and (width-min_w) % step_w == 0
                      and (height-min_h) % step_h == 0)
    if supported:
        print(width, height)
        break
PY
)
    if [[ -n "$front_selection" ]]; then
      read -r -a front_size <<< "$front_selection"
      front_camera_enabled=true
    else
      printf 'Front camera %s has no verified YUYV 640x480 or 320x240 mode; rear camera remains active. Check v4l2-ctl formats.\n' "$FRONT_CAMERA_DEVICE" >&2
    fi
  else
    printf 'Front camera format cannot be checked without v4l2-ctl; rear camera remains active.\n' >&2
  fi
fi
if command -v fuser >/dev/null; then
  for device in "$OPENCR_DEVICE" "$CAMERA_DEVICE"; do
    if fuser "$device" >/dev/null 2>&1; then
      printf 'Device already in use: %s. Stop the existing bringup/camera with Ctrl+C first.\n' "$device" >&2
      exit 2
    fi
  done
  if [[ "$front_camera_enabled" == true ]] && fuser "$FRONT_CAMERA_DEVICE" >/dev/null 2>&1; then
    printf 'Front camera already in use: %s; rear camera remains active.\n' "$FRONT_CAMERA_DEVICE" >&2
    front_camera_enabled=false
  fi
fi

children=()
required_children=()
front_child=0
cleanup() {
  trap '' INT TERM HUP
  trap - EXIT
  # Signal only process groups started by this script.
  local child tick alive
  for child in "${children[@]}"; do kill -INT -- "-$child" 2>/dev/null || true; done
  for ((tick=0; tick<80; tick++)); do
    alive=false
    for child in "${children[@]}"; do
      if kill -0 -- "-$child" 2>/dev/null; then alive=true; fi
    done
    if [[ "$alive" == false ]]; then break; fi
    sleep 0.1
  done
  for child in "${children[@]}"; do kill -TERM -- "-$child" 2>/dev/null || true; done
  for ((tick=0; tick<20; tick++)); do
    alive=false
    for child in "${children[@]}"; do
      if kill -0 -- "-$child" 2>/dev/null; then alive=true; fi
    done
    if [[ "$alive" == false ]]; then break; fi
    sleep 0.1
  done
  for child in "${children[@]}"; do kill -KILL -- "-$child" 2>/dev/null || true; done
  for child in "${children[@]}"; do wait "$child" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

printf 'Robot bringup + camera: domain=16 model=%s lidar=%s rear=%s (%sx%s), backend=%s, fps=%s, localization=%s\n' \
  "$TURTLEBOT3_MODEL" "$LDS_MODEL" "$CAMERA_DEVICE" "$rear_width" "$rear_height" "$CAMERA_BACKEND" "$CAMERA_PUBLISH_FPS" "$LOCALIZATION_MODE"
if [[ "$front_camera_enabled" == true ]]; then
  printf 'Front camera: %s -> /camera/front/image_raw (%sx%s YUYV)\n' \
    "$FRONT_CAMERA_DEVICE" "${front_size[0]}" "${front_size[1]}"
else
  printf 'Front camera: disabled or unavailable; /camera/front/image_raw will remain offline.\n' >&2
fi
printf 'Ctrl+C stops the robot bringup and cameras.\n'
if [[ "$ESTIMATION_LOCATION" == robot ]]; then
  geometry_args=()
  if [[ -n "${WHEEL_RADIUS:-}" ]]; then geometry_args+=(wheel_radius:="$WHEEL_RADIUS"); fi
  if [[ -n "${WHEEL_SEPARATION:-}" ]]; then geometry_args+=(wheel_separation:="$WHEEL_SEPARATION"); fi
  setsid ros2 launch smart_vision_amr hardware_robot_runtime.launch.py \
    localization_mode:="$LOCALIZATION_MODE" usb_port:="$OPENCR_DEVICE" \
    "${geometry_args[@]}" &
elif [[ "$LOCALIZATION_MODE" == ekf_independent ]]; then
  setsid ros2 launch "$LEGACY_EKF_LAUNCH" usb_port:="$OPENCR_DEVICE" &
else
  setsid ros2 launch turtlebot3_bringup robot.launch.py use_sim_time:=false usb_port:="$OPENCR_DEVICE" &
fi
children+=("$!")
required_children+=("$!")
# ROS remapping matches the full topic name: remapping image_raw does not also
# remap the image_transport plugin's image_raw/compressed topic.
if [[ "$CAMERA_BACKEND" == rate_limited ]]; then
  if [[ "$front_camera_enabled" == true ]]; then
    setsid python3 "$SCRIPT_DIR/ros_camera_compressed.py" \
      --device "$FRONT_CAMERA_DEVICE" \
      --topic /camera/front/image_raw/compressed --node-name front_camera_driver \
      --frame-id camera_rgb_optical_frame \
      --width "${front_size[0]}" --height "${front_size[1]}" \
      --fps "$CAMERA_PUBLISH_FPS" --jpeg-quality "$CAMERA_JPEG_QUALITY" \
      --pixel-format YUYV &
    children+=("$!")
    front_child=$!
  fi
  setsid python3 "$SCRIPT_DIR/ros_camera_compressed.py" \
    --device "$CAMERA_DEVICE" \
    --topic /camera/image_raw/compressed --node-name rear_camera_driver \
    --frame-id rear_camera_optical_frame \
    --width "$rear_width" --height "$rear_height" \
    --fps "$CAMERA_PUBLISH_FPS" --jpeg-quality "$CAMERA_JPEG_QUALITY" \
    --pixel-format YUYV &
else
  if [[ "$front_camera_enabled" == true ]]; then
    setsid ros2 run v4l2_camera v4l2_camera_node --ros-args \
      -r __node:=front_camera_driver \
      -r image_raw:=/camera/front/image_raw \
      -r image_raw/compressed:=/camera/front/image_raw/compressed \
      -r camera_info:=/camera/front/camera_info \
      -p video_device:="$FRONT_CAMERA_DEVICE" \
      -p pixel_format:=YUYV -p output_encoding:=rgb8 \
      -p image_raw.jpeg_quality:="$CAMERA_JPEG_QUALITY" \
      -p image_size:="[${front_size[0]}, ${front_size[1]}]" &
    children+=("$!")
    front_child=$!
  fi
  setsid ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -r image_raw:=/camera/image_raw \
    -r image_raw/compressed:=/camera/image_raw/compressed \
    -r camera_info:=/camera/camera_info \
    -p video_device:="$CAMERA_DEVICE" \
    -p pixel_format:=YUYV -p output_encoding:=rgb8 \
    -p image_raw.jpeg_quality:="$CAMERA_JPEG_QUALITY" \
    -p image_size:="[$rear_width, $rear_height]" &
fi
children+=("$!")
required_children+=("$!")

# Poll required launch parents so even an immediate startup failure is noticed.
# wait -n can miss a child that exited before the wait was registered.
while true; do
  for child in "${required_children[@]}"; do
    if ! kill -0 "$child" 2>/dev/null; then
      if wait "$child"; then result=1; else result=$?; fi
      printf 'Bringup or camera exited; stopping both (status %s).\n' "$result" >&2
      exit "$result"
    fi
  done
  if [[ "$front_child" -gt 0 ]] && ! kill -0 "$front_child" 2>/dev/null; then
    wait "$front_child" 2>/dev/null || true
    printf 'Front camera driver exited; rear camera and robot bringup remain active.\n' >&2
    front_child=0
  fi
  sleep 0.2
done
