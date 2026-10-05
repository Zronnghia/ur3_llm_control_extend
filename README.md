# ur3_llm_control — Bài 03: LLM Skill Planning với Gripper và Camera
ROS 2 Humble · MoveIt 2 · Gazebo Classic 11 · 9Router

> **Cài đặt và chạy từng lệnh: xem [`HUONG_DAN_CHAY.md`](HUONG_DAN_CHAY.md).**
> README này giải thích kiến trúc, logic xử lý và các quyết định thiết kế (dùng khi bảo vệ bài).

```
Natural Language Command
        ↓
Camera (detect_objects)  ──►  trạng thái bàn: khối nào ở đâu, zone nào trống / bị chiếm
        ↓
LLM Planner (9Router)   chỉ chọn skill + tham số + thứ tự
        ↓
Structured Plan (JSON)  ──►  Plan Validator (cú pháp, whitelist, tên, ngữ nghĩa)
        ↓
Skill Executor  (sắp xếp lại, tự dọn zone bị chiếm, tìm vị trí tạm, kiểm tra lại bằng camera)
        ↓
Robot Skills  (pick / place / check_zone / find_free_position / ...)
        ↓
MoveIt 2  (OMPL + Cartesian, kiểm tra va chạm cả gripper và khối đang cầm)
        ↓
UR3e + Gripper 2 ngón (Gazebo, gắp bằng lực kẹp + ma sát)  ──►  Camera kiểm tra kết quả
```

LLM **không bao giờ** thấy tên khớp, giá trị khớp, toạ độ hay quỹ đạo. Toàn bộ hình học
(độ cao tiếp cận, hướng kẹp, toạ độ vị trí tạm) nằm trong code và `config/scene.yaml`;
mọi chuyển động do MoveIt 2 lập kế hoạch và thực thi.

---

## 1. Môi trường mô phỏng (đúng yêu cầu mục 1)

| Thành phần | Hiện thực |
|---|---|
| 1 UR3e | `ur_description` (không sửa), include nguyên `ur.urdf.xacro` |
| 1 gripper | `urdf/parallel_gripper.xacro`: gripper 2 ngón song song gắn vào `tool0`, 2 khớp tịnh tiến, điều khiển **lực** qua ros2_control |
| 1 camera | camera RGB cố định phía trên bàn (Gazebo camera sensor + `gazebo_ros_camera`) → `/overhead_camera/image_raw` |
| 1 bàn | bàn xám trung tính (không trùng màu khối nào) |
| 3 zone | `zone_a`, `zone_b`, `zone_c`: ô trắng viền đen vẽ trên bàn |
| 5 block | `red_cube`, `yellow_cube`, `green_cube`, `blue_cube`, `purple_cube` — **vật rắn động lực học thật** (khối lượng 50 g, quán tính, trọng lực, ma sát) |

Bố trí ban đầu chính là tình huống demo đề yêu cầu: **`blue_cube` đang nằm trong Zone B**,
Zone A và C trống, 4 khối còn lại nằm rải trên bàn ngoài zone.

## 2. Gripper (mục 2) — gắp thật, không dịch chuyển vật

- Mỗi ngón là một link động học có khối lượng, nối vào thân gripper bằng khớp prismatic
  (`left_finger_joint`, `right_finger_joint`, 0 → 4.5 cm mỗi bên).
- Controller `gripper_controller` = `joint_trajectory_controller` với **giao diện lực**
  (`command_interfaces: [effort]`): mỗi ngón bị kéo về vị trí đích bởi một lực tỉ lệ sai số
  (PD, p = 400 N/m, giới hạn 20 N trong URDF). Lệnh `close` = đích 0 (hai ngón chạm nhau):
  gặp khối 4 cm thì mỗi ngón ép ≈ 400 × 0.02 = 8 N vào mặt khối; khối được giữ bởi
  **ma sát** (μ = 1.2–1.5) trong bộ giải vật lý ODE. Hai "lò xo" này còn giữ khối ở giữa
  hai ngón khi cánh tay di chuyển (thử bằng lực hằng số thì khối trôi 17 mm theo phương kẹp).
- Cánh tay dùng **giao diện vận tốc** (`joint_trajectory_controller`, command
  `velocity` + phản hồi vị trí). Lý do: với giao diện `position`, `gazebo_ros2_control`
  *dịch chuyển tức thời* các khớp (SetPosition + vận tốc 0), ngón tay "trượt xuyên" qua
  khối mà không kéo được nó — đã kiểm chứng: khối nằm lại trên bàn khi nâng. Với vận tốc,
  các link chuyển động thật và ma sát mang khối theo.
- **Không có dòng code nào đặt pose của khối** (không `set_entity_state`, không
  `SetModelState`, khối không phải link kinematic). Có test tự động quét toàn bộ mã nguồn
  để đảm bảo điều này (`test_nothing_teleports_objects`).
- Gắp có kiểm tra thật: sau khi đóng, khe hở hai ngón được đo từ `/joint_states`.
  Khe ≈ 4 cm (bề rộng khối) → đã kẹp được; khe ≈ 0 → `GRASP_FAILED` (kẹp trượt/không có vật).
  Sau khi nâng lên đo lại lần nữa để phát hiện vật tuột.
- Hướng kẹp: camera ước lượng góc xoay (yaw) của khối; gripper xoay theo mặt khối
  (`yaw_khối + k·90°`, chọn k làm các khớp di chuyển ít nhất). Khi đặt, khối được xoay
  thẳng hàng với trục bàn.

## 3. Camera và trạng thái môi trường (mục 3)

Không có vị trí khối nào được dùng để lập kế hoạch ngoài những gì camera thấy.
`objects.*.spawn_xy` trong `scene.yaml` **chỉ** dùng để sinh world Gazebo (như người đặt
vật lên bàn); planner không đọc giá trị này.

Pipeline thị giác (`vision.py`, numpy thuần, không cần OpenCV):

1. Ảnh RGB → HSV, mỗi màu khối một dải Hue (`scene.yaml: vision.hue_ranges`),
   ngưỡng bão hoà/độ sáng để loại bàn xám, zone trắng, robot đen.
2. Chỉ xét vùng ảnh nhìn thấy mặt bàn (ROI = 4 góc bàn chiếu qua mô hình camera).
3. Thành phần liên thông → blob lớn nhất của mỗi màu.
4. Từ blob ra toạ độ trên bàn bằng **mô hình camera thật**: K từ `/camera_info`,
   ngoại tham số từ TF `world → camera_optical_frame`:
   - yaw: hình chữ nhật diện tích nhỏ nhất của blob chiếu lên mặt phẳng mặt trên khối;
   - (x, y): chọn sao cho *hình chiếu của một khối 4 cm đặt tại (x, y, yaw)* có cùng
     trọng tâm với blob quan sát (Gauss–Newton trên 8 đỉnh khối). Cách này loại sai số
     phối cảnh (khối xa tâm ảnh bị camera thấy thêm một mặt bên).
5. Từ vị trí khối suy ra: khối nằm trong zone nào (`slot_of`), zone nào trống / bị
   chiếm bởi khối nào (`occupant`, tính theo **diện tích chân khối**, khối nằm lấn nửa
   zone vẫn tính là chiếm).

Camera cố định nên cánh tay phải ra khỏi tầm nhìn khi chụp: tư thế `home` được chọn để
toàn bộ cánh tay + gripper nằm ngoài vùng ảnh của bàn (được `check_scene` kiểm tra), và
mọi skill dùng camera tự đưa robot về `home` trước khi chụp; ảnh dùng phải có
timestamp **sau** thời điểm yêu cầu.

Ảnh kèm khung nhận dạng được phát trên `/llm_robot/camera_debug` (RViz hiển thị sẵn).

## 4. Skill (mục 4)

| Skill (LLM được dùng) | Loại | Việc làm |
|---|---|---|
| `detect_objects()` | cảm biến | về home, chụp, cập nhật vị trí mọi khối + planning scene MoveIt |
| `check_zone(zone)` | cảm biến | chụp lại, trả `FREE` hoặc `OCCUPIED by <khối>` |
| `find_object(object)` | cảm biến | chụp lại, trả zone hoặc toạ độ trên bàn, `NOT_FOUND` nếu không thấy |
| `find_free_position(object)` | tính toán | chọn một chỗ trống trên bàn để đặt tạm (xem dưới) |
| `pick(object)` | chuyển động | mở gripper → lên trên khối (IK, yaw theo mặt khối) → hạ thẳng → kẹp + kiểm tra khe → nâng thẳng → kiểm tra lại |
| `place(object, zone \| temporary_position)` | chuyển động | lên trên đích → hạ thẳng → mở gripper → nâng thẳng |
| `move_above(object)`, `move_to_zone(zone)`, `home()` | chuyển động | như Bài 02 |

Trạng thái trả về: `SUCCESS, SKIPPED, FAILED, INVALID_OBJECT, INVALID_ZONE, INVALID_STATE,
NOT_FOUND, GRASP_FAILED, PLANNING_FAILED, EXECUTION_FAILED` kèm chi tiết
(ví dụ `SUCCESS (OCCUPIED by blue_cube)`, `SUCCESS (grip 3.8 cm)`).

**`find_free_position`** — lưới 2 cm trên mặt bàn, loại các điểm:
cách mép bàn < 6 cm; nằm trong hoặc sát zone (cách viền zone < 5 cm — zone phải luôn
dùng được); cách khối khác < 10 cm (ngón gripper mở rộng 11.4 cm cần chỗ); ngoài vành
tầm với (0.20–0.46 m). Các điểm còn lại sắp theo khoảng cách tới vị trí hiện tại của khối
(di chuyển ngắn nhất), điểm đầu tiên **IK được cả tư thế tiếp cận lẫn tư thế đặt** được chọn.

## 5. Logic xử lý khi zone đích bị chiếm (mục 4 + 6)

Ví dụ đề bài — Zone B có `blue_cube`, người dùng: *"Put the red cube in Zone B."*

1. **Camera trước LLM.** `detect_objects()` chạy trước khi gọi LLM; user prompt gửi cho
   LLM có dạng:
   ```
   CURRENT STATE (camera):
     - red_cube: on the table, not in a zone
     - blue_cube: in zone_b
     ...
   Zones:
     - zone_a: FREE
     - zone_b: OCCUPIED by blue_cube
   Command: Put the red cube in Zone B.
   ```
2. **LLM lập kế hoạch có kiểm tra zone** (luật trong system prompt: gọi `check_zone`
   trước khi đặt vào zone; nếu zone bị khối khác chiếm thì chuyển khối đó ra
   `temporary_position` trước):
   ```
   check_zone(zone_b)
   pick(blue_cube)
   find_free_position(blue_cube)
   place(blue_cube, temporary_position)
   pick(red_cube)
   place(red_cube, zone_b)
   home()
   ```
3. **Validator** kiểm tra kế hoạch trên bản sao trạng thái camera (không pick khối camera
   không thấy, không place khi không cầm, không kết thúc khi còn cầm...).
4. **Executor — lưới an toàn tất định.** Trước khi chạy, kế hoạch được mô phỏng trên
   trạng thái camera: nếu LLM *quên* dọn zone, executor tự chèn
   `pick(blue_cube) / find_free_position(blue_cube) / place(blue_cube, temporary_position)`
   (đánh dấu `[auto]`); khối đã đúng zone thì `SKIPPED`; các khối pick/place độc lập được
   sắp lại để khối nào có zone đích trống đi trước (ít phải đặt tạm hơn); vị trí tạm được
   tính sẵn và in ra.
5. **Vòng kín với camera khi chạy.** `check_zone(zone_b)` chụp lại thật. Nếu camera thấy
   trạng thái khác dự đoán (có ai đặt thêm khối, khối bị xô...), các bước còn lại được
   **lập lại từ ảnh mới**. `place` còn kiểm tra lần cuối zone đích trống trong mô hình thế
   giới trước khi hạ xuống.
6. **Kiểm tra kết quả bằng camera.** Sau `home()`, chụp lại và so với kế hoạch:
   `red_cube in zone_b ... OK (3 mm from centre)`, `blue_cube on the table ... OK`.
   Sai → `TASK FAILED`.

Output terminal đầy đủ của tình huống này (chạy thật trong Gazebo) ở mục 8.

## 6. Yêu cầu về LLM (mục 5)

- LLM chỉ: hiểu lệnh (Việt/Anh), chọn skill trong whitelist, điền tham số là **tên**
  khối/zone, sắp thứ tự. Mọi khoá lạ (`joints`, `trajectory`, `pose`, `x`, `y`, `z`,
  `coord...`) bị validator chặn với lỗi *"low-level robot control is forbidden"*.
- Kế hoạch **luôn** qua validator trước khi chạy; bị từ chối thì lỗi được gửi lại cho LLM
  một lần, vẫn sai → `TASK REJECTED`, robot đứng yên.
- Nhiệm vụ cá nhân hoá theo MSSV giữ nguyên Bài 02 (P = 54 mod 6 = 0 → A đỏ, B vàng, C xanh
  dương); với 5 khối, khối không thuộc bảng (xanh lá, tím) nằm trong zone sẽ được dời ra
  vị trí tạm.

## 7. MoveIt 2, Plan Validator, collision checking (giữ như Bài 02)

- `launch/moveit.launch.py`: `move_group` với mô hình **UR3e + gripper**
  (`urdf/ur_gripper.urdf.xacro`, `srdf/ur_gripper.srdf`) → MoveIt kiểm tra va chạm của
  cả ngón gripper, không chỉ cánh tay.
- Planning scene: sàn, bàn, **các khối camera thấy** (cập nhật mỗi lần chụp), khối đang
  kẹp được *attach* vào `tool0` tại tâm kẹp.
- Chuyển động tự do: IK số (tool hướng xuống, đúng yaw kẹp) **seed từ trạng thái khớp
  hiện tại** → mục tiêu khớp duy nhất (dung sai 0.001 rad) gửi cho OMPL (tránh nhảy nhánh
  động học, lỗi quay cổ tay 2π ở Bài 02). Lên/xuống: Cartesian path, `avoid_collisions=true`.
- Trước đoạn hạ cuối, khối mục tiêu được tạm bỏ khỏi planning scene (ngón mở sẽ bao quanh
  nó), sau đó được attach; khi thả, khối được thêm lại vào scene sau khi gripper đã rút lên.

## 8. Kết quả chạy thử (Gazebo + MoveIt 2 thật)

Output thật của tình huống demo đề yêu cầu (Zone B có `blue_cube`, lệnh
*"Put the red cube in Zone B."*), 66 s, trong môi trường kiểm thử Ubuntu 22.04 + ROS 2 Humble
+ Gazebo 11 + MoveIt 2.5.10 (cùng phiên bản apt). **LLM ở lần chạy này là server giả lập
9Router** trả kế hoạch đúng như prompt yêu cầu — khi nộp bài hãy thay bằng log chạy với
model thật trên máy bạn:

```
USER COMMAND:
Put the red cube in Zone B.

CAMERA (detect_objects):
  red_cube     table (0.220, -0.159)
  yellow_cube  table (0.220, 0.001)
  green_cube   table (0.220, 0.161)
  blue_cube    zone_b
  purple_cube  table (0.301, -0.299)
  zones        zone_a: FREE, zone_b: OCCUPIED (blue_cube), zone_c: FREE

LLM PLAN:
check_zone(zone_b)
pick(blue_cube)
find_free_position(blue_cube)
place(blue_cube, temporary_position)
pick(red_cube)
place(red_cube, zone_b)
home()

JSON PLAN:
{"plan": [{"skill": "check_zone", "zone": "zone_b"}, {"skill": "pick", "object": "blue_cube"}, {"skill": "find_free_position", "object": "blue_cube"}, {"skill": "place", "object": "blue_cube", "zone": "temporary_position"}, {"skill": "pick", "object": "red_cube"}, {"skill": "place", "object": "red_cube", "zone": "zone_b"}, {"skill": "home"}]}

EXECUTION:
check_zone(zone_b) ................ SUCCESS (OCCUPIED by blue_cube)
pick(blue_cube) ................... SUCCESS (grip 3.8 cm)
find_free_position(blue_cube) ..... SUCCESS ((0.360, 0.280))
place(blue_cube, temporary_position) .. SUCCESS (released at (0.360, 0.280))
pick(red_cube) .................... SUCCESS (grip 3.8 cm)
place(red_cube, zone_b) ........... SUCCESS (released at zone_b)
home() ............................ SUCCESS

VERIFY (camera):
blue_cube on the table ............ OK at (0.361, 0.281)
red_cube in zone_b ................ OK (2 mm from centre)

TASK SUCCESS
World state (camera):
  red_cube     zone_b
  yellow_cube  table (0.220, 0.001)
  green_cube   table (0.220, 0.161)
  blue_cube    table (0.361, 0.281)
  purple_cube  table (0.301, -0.299)
  zones        zone_a: FREE, zone_b: OCCUPIED (red_cube), zone_c: FREE
```

Vị trí thật của khối đọc từ Gazebo sau khi chạy (chỉ dùng để kiểm thử, hệ thống không đọc):
`red_cube` (0.400, −0.001) — lệch tâm Zone B 1 mm; `blue_cube` (0.360, 0.279) — đúng chỗ
`find_free_position` đã chọn; hai khối đều đứng thẳng (nghiêng 0.1°).

## 9. Kiểm thử

| Mức | Cách chạy | Kết quả |
|---|---|---|
| Unit + tích hợp (106 test, không cần ROS) | `python3 -m pytest test -q` | nhận dạng màu/vị trí/yaw trên ảnh tổng hợp (sai số < 3 mm, yaw < 4°, 12 bố trí ngẫu nhiên), IK có yaw, validator (26 kế hoạch sai bị từ chối), executor tự dọn zone / sắp xếp lại / bỏ qua, vòng kín camera (có người thả khối vào zone giữa chừng), pipeline đầy đủ với robot + camera giả lập, gắp trượt → `GRASP_FAILED`, kiểm tra cuối phát hiện khối bị xô, client HTTP 9Router, không file nào dịch chuyển vật |
| Tổng hợp | `tools/run_all_checks.sh` | + flake8, `check_scene` (UR3 & UR3e), world XML, cấu hình, package ROS, URDF UR3e + gripper |
| **End-to-end trong Gazebo** | `sim.launch.py` + `llm_robot_node` (LLM thay bằng server giả lập 9Router trả kế hoạch như một LLM thật) | 7 kịch bản, vị trí thật của khối được đọc từ Gazebo để đối chiếu: |

Các kịch bản end-to-end đã chạy (mỗi kịch bản chạy lại node, trạng thái Gazebo nối tiếp):

| Lệnh | Tình huống | Kết quả (vị trí thật trong Gazebo) |
|---|---|---|
| Put the red cube in Zone B. | B bị `blue_cube` chiếm | TASK SUCCESS, đỏ trong B (lệch 1 mm), xanh ra vị trí tạm |
| Arrange all objects according to my student ID. | đỏ đang ở B | TASK SUCCESS, A đỏ / B vàng / C xanh dương (lệch ≤ 1.2 mm) |
| Put the green cube in zone C. | C bị xanh dương chiếm | TASK SUCCESS, xanh dương ra vị trí tạm, xanh lá vào C |
| *(LLM "quên" dọn zone)* put the purple cube in zone B | B bị chiếm, kế hoạch chỉ có pick/place | executor tự chèn 3 bước `[auto]`, TASK SUCCESS |
| Put the yellow cube in zone A. | khối vàng bị xoay 35°, khối xanh lá nằm lấn mép Zone A | camera đo yaw 35.0°, báo A bị chiếm; gripper xoay theo khối, đặt thẳng hàng vào A |
| Move the orange cube to zone A. | khối không tồn tại | TASK REJECTED, robot không chuyển động |
| Rotate the shoulder joint to 90 degrees. | lệnh khớp | validator chặn 2 lần, TASK REJECTED |

Chế độ một terminal (`llm_robot.launch.py` + `send_command`, kết quả trên `/llm_robot/result`)
cũng đã chạy thành công.

**Giới hạn trung thực:** bộ kiểm thử Gazebo dùng LLM giả lập (máy kiểm thử không có API key
LLM thật). Prompt, validator và executor đã được thiết kế để chịu lỗi của LLM thật (alias tham
số, tự dọn zone nếu LLM quên, thử lại một lần khi kế hoạch sai), nhưng nên thử trước bằng
`offline_cli` với model thật. RViz chỉ được kiểm tra khởi động (không có plugin MoveIt
trên máy kiểm thử); các display thêm vào (MarkerArray, Image) nạp bình thường.

## 10. Cấu trúc package

```
ur3_llm_control/
├── config/
│   ├── scene.yaml               bàn, 5 khối (màu + vị trí spawn), 3 zone, camera, gripper, chuyển động
│   ├── student_config.yaml      họ tên, MSSV, 9Router
│   ├── ros2_controllers.yaml    arm JTC + joint_state_broadcaster + gripper_controller (effort)
│   ├── initial_positions.yaml   robot xuất hiện ở tư thế home
│   └── kinematics.yaml, ompl_planning.yaml, moveit_controllers.yaml, moveit_joint_limits.yaml
├── urdf/  ur_gripper.urdf.xacro, parallel_gripper.xacro
├── srdf/  ur_gripper.srdf
├── rviz/  view_robot.rviz       MoveIt + zone/khối camera thấy + ảnh camera
├── launch/
│   ├── sim.launch.py            Gazebo + UR3e + gripper + camera + controllers + MoveIt + RViz
│   ├── moveit.launch.py         move_group với mô hình có gripper
│   └── llm_robot.launch.py      sim + node (lệnh qua topic)
├── ur3_llm_control/
│   ├── vision.py                phân đoạn màu, liên thông, ước lượng pose khối (numpy)
│   ├── perception.py            ROS: ảnh + camera_info + TF → vision, ảnh debug
│   ├── robot_skills.py          skill cảm biến + chuyển động + gripper
│   ├── skill_executor.py        mở rộng kế hoạch, dọn zone, vòng kín camera, kiểm tra cuối
│   ├── task_validator.py        cổng an toàn
│   ├── llm_planner.py           prompt (có trạng thái camera) + client 9Router + retry
│   ├── scene_model.py           cấu hình + trạng thái thế giới + find_free_position
│   ├── moveit_backend.py        MoveIt 2 + gripper controller + planning scene
│   ├── ur_kinematics.py         FK/IK UR (tool hướng xuống, có yaw)
│   ├── pipeline.py              in USER COMMAND / CAMERA / LLM PLAN / EXECUTION / VERIFY
│   ├── llm_robot_node.py        node ROS 2 (terminal tương tác + topic)
│   ├── camera_snapshot.py       lưu ảnh camera + in kết quả nhận dạng
│   ├── world_gen.py             sinh world Gazebo từ scene.yaml
│   ├── check_scene.py           kiểm tra offline tầm với, camera, home không che bàn
│   ├── fake_backend.py, synthetic_camera.py   robot + camera giả lập để test không cần ROS
│   └── offline_cli.py           LLM thật + robot/camera giả lập
├── test/  test_core.py, test_vision.py
└── tools/ run_all_checks.sh
```
