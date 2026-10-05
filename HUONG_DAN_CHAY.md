# HƯỚNG DẪN CÀI ĐẶT VÀ CHẠY — ur3_llm_control (Bài 03: gripper + camera)

- Hệ điều hành: **Ubuntu 22.04** (chạy trực tiếp, không WSL)
- ROS 2 Humble · MoveIt 2 · Gazebo Classic 11 · 9Router
- Workspace: **`/workspaces/ur_gazebo`**, package ở **`/workspaces/ur_gazebo/src/ur3_llm_control`**

> Máy đã chạy được Bài 02 thì **bỏ qua Bước 1–3 và Bước 6** (ROS, UR, MoVeIt, Gazebo, 9Router
> đã có). Chỉ cần làm Bước 4 (thay package), 5 (build), 7–9.
>
> Nếu workspace thực ra là `~/workspaces/ur_gazebo` thì thay `/workspaces/ur_gazebo` bằng
> `~/workspaces/ur_gazebo` ở mọi lệnh. Đường dẫn **không được có dấu cách**.

---

## Bước 1 — Kiểm tra hệ thống

```bash
lsb_release -a                 # Ubuntu 22.04 (jammy)
echo $XDG_SESSION_TYPE         # x11 là tốt nhất cho Gazebo; wayland xem Bước 10
free -h                        # nên có >= 8 GB RAM
```

## Bước 2 — ROS 2 Humble (bỏ qua nếu đã có `/opt/ros/humble`)

```bash
sudo apt update && sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install -y ros-humble-desktop ros-dev-tools
```

```bash
# LUÔN chạy (kể cả khi đã có ROS): các gói ros-humble-* phải cùng một đợt cập nhật
sudo apt update && sudo apt upgrade -y
```

## Bước 3 — UR, MoveIt 2, Gazebo, ros2_control

```bash
sudo apt install -y \
  ros-humble-ur \
  ros-humble-moveit \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  python3-yaml python3-numpy python3-pytest git
sudo rosdep init        # nếu báo "already exists" thì bỏ qua
rosdep update
```

`ros-humble-gazebo-ros-pkgs` có sẵn plugin camera (`gazebo_plugins`); `ros-humble-ros2-controllers`
có `joint_trajectory_controller` dùng cho cả cánh tay và gripper. Bài 03 **không còn cần**
`Universal_Robots_ROS2_Gazebo_Simulation` (package tự mang file controller riêng); nếu đã
clone ở Bài 02 thì cứ để đó, không ảnh hưởng.

## Bước 4 — Đặt package Bài 03 vào workspace

```bash
mkdir -p /workspaces/ur_gazebo/src
cd /workspaces/ur_gazebo/src
# cất bản Bài 02 đi (không xoá, để còn đối chiếu)
[ -d ur3_llm_control ] && mv ur3_llm_control ~/ur3_llm_control_bai02_$(date +%Y%m%d_%H%M)
tar -xJf ~/Downloads/ur3_llm_control_bai03.tar.xz -C /workspaces/ur_gazebo/src
ls /workspaces/ur_gazebo/src/ur3_llm_control
# config  HUONG_DAN_CHAY.md  launch  package.xml  README.md  resource  rviz  setup.cfg  setup.py  srdf  test  tools  ur3_llm_control  urdf
```

> `package.xml` phải nằm ngay trong `src/ur3_llm_control/` (không lồng hai lần).

## Bước 5 — Build

```bash
cd /workspaces/ur_gazebo
source /opt/ros/humble/setup.bash
rm -rf build/ur3_llm_control install/ur3_llm_control      # xoá kết quả build của bản cũ
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source /workspaces/ur_gazebo/install/setup.bash
```

```bash
# tự source mỗi khi mở terminal (chỉ cần 1 lần)
grep -q "/opt/ros/humble/setup.bash" ~/.bashrc || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
grep -q "/workspaces/ur_gazebo/install/setup.bash" ~/.bashrc || echo "source /workspaces/ur_gazebo/install/setup.bash" >> ~/.bashrc
```

```bash
ros2 pkg executables ur3_llm_control
# ur3_llm_control camera_snapshot
# ur3_llm_control check_scene
# ur3_llm_control llm_robot_node
# ur3_llm_control offline_cli
# ur3_llm_control send_command
```

## Bước 6 — 9Router (đã làm ở Bài 02 thì bỏ qua)

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
source ~/.bashrc && nvm install 22 && npm install -g 9router
```

1. (Tuỳ provider) đăng nhập / thêm provider trong dashboard 9Router.
2. Terminal A (để nguyên suốt buổi): `9router --no-browser --host 127.0.0.1`
3. Trình duyệt: http://localhost:20128 → **Providers** kiểm tra provider của model bạn dùng đang bật;
   **Endpoint** → copy API key của 9Router (`sk-...`).
4. Terminal mới:
```bash
echo 'export NINEROUTER_API_KEY="sk-DAN_KEY_9ROUTER_VAO_DAY"' >> ~/.bashrc
echo 'export NINEROUTER_MODEL="oc/muse-spark-1.2-contributor-free"' >> ~/.bashrc
source ~/.bashrc
```

## Bước 7 — Thông tin sinh viên

`config/student_config.yaml`: `student_name`, `student_id` (hiện là `Le Trong Nghia`,
`23020754` → P = 0 → Zone A đỏ, Zone B vàng, Zone C xanh dương), để `api_key: ""`.
Sửa xong thì build lại:
```bash
cd /workspaces/ur_gazebo && colcon build --symlink-install --packages-select ur3_llm_control
```

## Bước 8 — Kiểm tra offline (chưa cần Gazebo)

```bash
/workspaces/ur_gazebo/src/ur3_llm_control/tools/run_all_checks.sh
# dòng cuối phải là: ALL CHECKS PASSED
```
Gồm: cú pháp + flake8, **106 test** (camera/nhận dạng màu trên ảnh tổng hợp, IK có hướng
kẹp, validator, executor dọn zone, vòng kín camera, pipeline đầy đủ với robot + camera giả
lập), tầm với + camera nhìn đủ bàn + tư thế home không che camera, world XML, cấu hình LLM,
các package ROS cần thiết, URDF UR3e + gripper.

Thử LLM thật với robot + camera giả lập (không cần Gazebo):
```bash
ros2 run ur3_llm_control offline_cli "Put the red cube in Zone B."
ros2 run ur3_llm_control offline_cli "Đặt khối màu đỏ vào vùng B."
ros2 run ur3_llm_control offline_cli "Arrange all objects according to my student ID."
ros2 run ur3_llm_control offline_cli "Move the orange cube to zone A."   # phải TASK REJECTED
```

## Bước 9 — Chạy mô phỏng và demo

| Terminal | Việc |
|---|---|
| A | `9router --no-browser --host 127.0.0.1` |
| B | Gazebo + UR3e + gripper + camera + MoveIt 2 + RViz |
| C | node điều khiển bằng ngôn ngữ tự nhiên |

### 9.1 Terminal B

```bash
ros2 launch ur3_llm_control sim.launch.py
```
Đợi lần lượt:
```
[sim.launch] Gazebo world generated: /tmp/ur3_llm_world.world
... Configured and activated joint_state_broadcaster
... Configured and activated joint_trajectory_controller
... Configured and activated gripper_controller
[sim.launch] controllers up -> starting MoveIt 2
... You can start planning now!
```
Gazebo có bàn xám, 3 ô trắng (Zone A trái / B giữa / C phải khi nhìn từ robot), 5 khối,
**khối xanh dương nằm trong Zone B**, camera treo trên bàn. RViz có thêm ô ảnh
"Camera (detections)" (trống cho đến khi node chạy).

Tuỳ chọn: `gazebo_gui:=false` (nhẹ hơn), `launch_rviz:=false`, `ur_type:=ur3`.

### 9.2 Terminal C

```bash
ros2 run ur3_llm_control llm_robot_node --ros-args -p use_sim_time:=true
```
Node mở gripper, về `home`, chụp ảnh và in những gì camera thấy, rồi hiện `Command>`.

### 9.3 Demo bắt buộc: zone đích đang bị chiếm

```
Command> Put the red cube in Zone B.
```
Kết quả mong đợi: camera báo `zone_b: OCCUPIED (blue_cube)` → LLM PLAN có
`check_zone(zone_b)`, `pick(blue_cube)`, `find_free_position(blue_cube)`,
`place(blue_cube, temporary_position)`, `pick(red_cube)`, `place(red_cube, zone_b)`, `home()`
→ robot dời khối xanh ra chỗ trống, gắp khối đỏ đặt vào B → `VERIFY (camera)` → `TASK SUCCESS`.
Trong Gazebo có thể thấy hai ngón kẹp khối và nhấc lên thật.

### 9.4 Các lệnh khác nên demo

```
Command> Arrange all objects according to my student ID.
Command> Sắp xếp các khối theo mã số sinh viên của tôi.
Command> Put the green cube in zone C.          (C đang có khối khác -> dọn trước)
Command> Where are the blocks?                  (chỉ quan sát)
Command> state                                  (in trạng thái camera)
Command> Move the orange cube to zone A.        (không có khối cam -> TASK REJECTED)
Command> Rotate the shoulder joint to 90 degrees.   (lệnh khớp -> bị validator chặn)
```

### 9.5 Xem camera

- RViz: display **Camera (detections)** hiện ảnh camera kèm khung quanh khối nhận dạng được
  và viền zone; display **Zones + detected blocks** hiện nhãn khối tại vị trí camera tính ra.
- Hoặc lưu ảnh (robot nên ở home):
```bash
ros2 run ur3_llm_control camera_snapshot --ros-args -p use_sim_time:=true -p out:=/tmp/cam.png
```

### 9.6 Đặt lại cảnh

Khối là vật thật trong Gazebo, không có lệnh "teleport" nào (đúng yêu cầu đề). Muốn về bố
trí ban đầu thì tắt Terminal B và C (`Ctrl+C`) rồi chạy lại 9.1 và 9.2.

### 9.7 Một terminal (lệnh qua topic)

```bash
ros2 launch ur3_llm_control llm_robot.launch.py            # Terminal B
ros2 run ur3_llm_control send_command "Put the red cube in Zone B."   # Terminal C
ros2 topic echo /llm_robot/result
```

### 9.8 Tắt

`Ctrl+C` ở C, rồi B, rồi A. Gazebo còn chạy ngầm: `pkill -9 -f gzserver; pkill -9 -f gzclient`.

---

## Bước 10 — Xử lý sự cố

| Hiện tượng | Cách xử lý |
|---|---|
| `Package 'ur3_llm_control' not found` | Terminal chưa `source /workspaces/ur_gazebo/install/setup.bash`. |
| `Could not load controller ... gripper_controller` / `joint_trajectory_controller` | Thiếu `ros-humble-ros2-controllers` hoặc chưa `apt upgrade` đồng bộ (Bước 2.4, 3). |
| `[setup] ERROR: no image on /overhead_camera/image_raw` | Camera cần OpenGL. Máy ảo / card yếu: chạy Terminal B với `export LIBGL_ALWAYS_SOFTWARE=1`. Kiểm tra `ros2 topic hz /overhead_camera/image_raw`. |
| `detect_objects() ... SUCCESS (4/5 blocks detected)` | Một khối bị che hoặc rơi khỏi bàn. Xem ảnh `camera_snapshot` / RViz. |
| `no camera image newer than the request (use_sim_time:=true missing?)` | Chạy node với `--ros-args -p use_sim_time:=true`. |
| `GRASP_FAILED (fingers closed to 0.0 cm: nothing grasped)` | Khối trượt/bị xô trước khi kẹp. Xem Gazebo; chạy `state` rồi thử lại lệnh (camera sẽ cập nhật vị trí mới). |
| `EXECUTION ABORTED: ... no free spot on the table` | Bàn quá chật; dời bớt khối (lệnh khác) hoặc nới `free_space` trong `scene.yaml`. |
| `[setup] ERROR: not available: ['/move_action' ...]` | MoveIt chưa lên. Đợi `You can start planning now!` rồi chạy lại node. |
| `PLANNING_FAILED` | Tăng `planning_time` trong `config/scene.yaml`, build lại; chạy `ros2 run ur3_llm_control check_scene`. |
| `cannot reach 9Router` / `HTTP 401` / `HTTP 429` | Terminal A chưa chạy / sai key 9Router / hết quota (đợi 1–2 phút hoặc `oc/muse-spark-1.2-contributor-free-lite`). |
| `REJECTED BY VALIDATOR` với lệnh hợp lệ | Model trả sai định dạng; thử model khác: `-p llm_model:=<alias>/<model>`. Executor vẫn tự dọn zone nếu LLM quên. |
| Cửa sổ Gazebo/RViz lỗi hiển thị (Wayland) | Chọn **Ubuntu on Xorg** ở màn hình đăng nhập, hoặc `export QT_QPA_PLATFORM=xcb`. |
| Gazebo đen màn hình / crash (máy ảo) | `export LIBGL_ALWAYS_SOFTWARE=1` rồi chạy lại; hoặc `gazebo_gui:=false`. |
| Robot RViz và Gazebo lệch | Quên `-p use_sim_time:=true` khi chạy node. |
| Log có `[gazebo_ros2_control]: Parameter 'hold_joints' has already been declared` | Vô hại: có 2 hệ ros2_control (cánh tay + gripper), plugin khai báo tham số này 2 lần. |
| Log có `No 3D sensor plugin(s) defined for octomap updates` | Vô hại: MoveIt không dùng cảm biến độ sâu (camera dùng riêng cho nhận dạng khối). |
| Muốn xem chi tiết chuyển động | Chạy node với `UR3_LLM_DEBUG=1 ros2 run ...`: in sai số khớp sau mỗi chuyển động và vị trí tool0 khi kẹp. |
