# TÀI LIỆU HƯỚNG DẪN HỌC MOTION PLANNING & CODEBASE ROS MOTION PLANNING

Tài liệu này tổng hợp toàn bộ kiến trúc, bản đồ chi phí (**Costmap**, **Global Map**, **Local Map**), cách vận hành và cách thay đổi/nghiên cứu các thuật toán lập kế hoạch đường đi toàn cục (**Global Planner**) trong repository này.

---

## MỤC LỤC
1. [Tổng quan Kiến trúc Repository](#1-tổng-quan-kiến-trúc-repository)
2. [Bản đồ và Bản đồ chi phí (Costmap: Global vs Local)](#2-bản-đồ-và-bản-đồ-chi-phí-costmap-global-vs-local)
3. [Cơ chế hoạt động của Global Planner trong Code](#3-cơ-chế-hoạt-động-của-global-planner-trong-code)
4. [Cách thay đổi thuật toán khi muốn chạy mô phỏng](#4-cách-thay-đổi-thuật-toán-khi-muốn-chạy-mô-phỏng)
5. [Mổ xẻ chi tiết code thuật toán A* (astar_planner.cpp)](#5-mổ-xẻ-chi-tiết-code-thuật-toán-a-astar_plannercpp)
6. [Hướng dẫn tự viết hoặc thêm một thuật toán mới](#6-hướng-dẫn-tự-viết-hoặc-thêm-một-thuật-toán-mới)
7. [Lộ trình học tập đề xuất](#7-lộ-trình-học-tập-đề-xuất)

---

## 1. TỔNG QUAN KIẾN TRÚC REPOSITORY

Hệ sinh thái mã nguồn được tổ chức thành các tầng rõ ràng:

```
ros_motion_planning/
├── scripts/                # Tập lệnh tiện ích: build.sh, main.sh, killpro.sh
├── huong dan chay          # Hướng dẫn khởi động mô phỏng nhanh
├── src/
│   ├── core/               # LÕI CÁC THUẬT TOÁN (Được viết bằng C++)
│   │   ├── path_planner/   # Chứa toàn bộ Global Planner (A*, RRT, JPS, ACO, GA,...)
│   │   ├── controller/     # Chứa các Local Planner / Controller (RPP, DWA, MPC, PID, LQR)
│   │   ├── common/         # Cấu trúc dữ liệu hình học, đồ thị, math helper, visualizer
│   │   └── system_config/  # Quản lý cấu hình bằng Protobuf
│   ├── sim_env/            # MÔI TRƯỜNG MÔ PHỎNG (Gazebo Fortress + RViz2 + Nav2)
│   │   ├── maps/           # Các bản đồ tĩnh (.yaml, .pgm)
│   │   ├── worlds/         # Thế giới 3D Gazebo (.world)
│   │   ├── urdf/           # Mô hình các robot (Turtlebot3 Waffle, Burger, Nanocar)
│   │   ├── config/         # Cấu hình Nav2, Costmap, AMCL
│   │   └── launch/         # Các launch file tự động hóa
│   ├── user_config/        # NƠI NGƯỜI DÙNG TÙY BIẾN THUẬT TOÁN VÀ ROBOT
│   └── plugins/            # Các plugin mở rộng cho Costmap (Voronoi), RViz, Gazebo
```

---

## 2. BẢN ĐỒ VÀ BẢN ĐỒ CHI PHÍ (COSTMAP: GLOBAL VS LOCAL)

### 2.1. Global Map (Bản đồ tĩnh toàn cục)
* **Vị trí**: `src/sim_env/maps/<tên_map>/` (ví dụ: `warehouse/warehouse.yaml` & `warehouse.pgm`).
* **Bản chất**: Bản đồ lưới chiếm chỗ (Occupancy Grid Map):
  * **0**: Ô trống hoàn toàn (Free Space).
  * **100**: Vật cản cố định như tường, kệ hàng (Lethal Obstacle).
  * **-1**: Khu vực chưa thám hiểm (Unknown).

### 2.2. Costmap (Bản đồ chi phí)
Bản đồ tĩnh chỉ cho biết có vật cản hay không. Nếu robot đi sát vào mép vật cản, nó sẽ rất dễ va quệt do sai số cơ khí hoặc trôi dạt định vị. 
Vì vậy, hệ thống chuyển bản đồ chiếm chỗ thành **Costmap** với giá trị từ **0 đến 254**:
* **0 (Free Space)**: Đi lại an toàn.
* **1 – 252 (Inflation Zone)**: Vùng an toàn nở rộng quanh vật cản. Càng gần vật cản, chi phí càng cao theo quy luật suy giảm hàm mũ:
  * Công thức: `cost = exp(-alpha * (distance - r_inscribed))`
* **253 (Inscribed Obstacle)**: Khoảng cách từ tâm robot đến vật cản nhỏ hơn bán kính robot. Robot đi vào đây chắc chắn sẽ va chạm mép.
* **254 (Lethal Obstacle)**: Điểm va chạm trực tiếp với vật cản thực tế.
* **255 (No Information)**: Ô chưa có dữ liệu quan sát.

### 2.3. Cấu trúc các tầng (Costmap Layers)
Cả Global Costmap và Local Costmap đều được ghép từ nhiều tầng dữ liệu:
1. **Static Layer**: Đọc từ bản đồ tĩnh được lưu từ trước (`map_server`).
2. **Obstacle Layer**: Cập nhật vật cản động thời gian thực từ cảm biến LiDAR qua topic `/scan`.
3. **Inflation Layer**: Nở rộng vùng an toàn dựa trên bán kính robot (`robot_radius` hoặc `footprint`).
4. **Voronoi Layer** *(tùy chọn)*: Tạo các "xương sống" (Generalized Voronoi Diagram) đi giữa các chướng ngại vật giúp robot chọn lối đi cách đều vật cản.

### 2.4. So sánh Global Costmap và Local Costmap

| Thuộc tính | Global Costmap | Local Costmap |
| :--- | :--- | :--- |
| **Hệ quy chiếu (`global_frame`)** | `map` (tọa độ cố định toàn cầu) | `odom` (gốc tọa độ đo đếm vị trí tương đối) |
| **Kích thước** | Toàn bộ bản đồ (Cố định, ví dụ 50m x 50m) | Cửa sổ trượt di động (`rolling_window: true`, ví dụ 3m x 3m) theo robot |
| **Tần số cập nhật** | Thấp (1.0 - 2.0 Hz) để tiết kiệm CPU | Cao (10.0 - 20.0 Hz) để phản ứng nhanh với vật cản |
| **Mục đích** | **Global Planner** tìm lộ trình từ điểm xuất phát đến đích | **Local Planner (Controller)** né vật cản động, sinh vận tốc `cmd_vel` |
| **File cấu hình** | `src/sim_env/config/nav2_params.yaml` (khối `global_costmap`) | `src/sim_env/config/nav2_params.yaml` (khối `local_costmap`) |

---

## 3. CƠ CHẾ HOẠT ĐỘNG CỦA GLOBAL PLANNER TRONG CODE

Tất cả các thuật toán lập kế hoạch toàn cục nằm tại `src/core/path_planner/path_planner`.

### 3.1. Lớp cơ sở trừu tượng: `PathPlanner`
* **File Header**: `include/path_planner/path_planner.h`
* **File Source**: `src/path_planner.cpp`

Mọi giải thuật (A*, RRT, Dijkstra, JPS, ACO...) đều kế thừa từ class `PathPlanner` và phải cài đặt hàm cốt lõi:
```cpp
virtual bool plan(const common::geometry::Point3d& start,
                  const common::geometry::Point3d& goal,
                  common::geometry::Points3d* path,
                  common::geometry::Points3d* expand) = 0;
```
* **Đầu vào**:
  * `start`, `goal`: Điểm xuất phát và đích đến trong hệ tọa độ thế giới thực `(x, y, yaw)`.
  * Dữ liệu bản đồ chi phí thông qua con trỏ `costmap_` (`costmap_2d::Costmap2D*`).
* **Đầu ra**:
  * `path`: Danh sách các điểm liên tiếp nhau tạo thành quỹ đạo toàn cục truyền xuống cho Controller đi theo.
  * `expand`: Danh sách các điểm/node mà thuật toán đã duyệt qua trong quá trình tìm kiếm (được publish lên topic `/expand` để trực quan hóa hiệu quả thuật toán trên RViz).

Các hàm bổ trợ có sẵn trong `PathPlanner`:
* `world2Map(wx, wy, mx, my)`: Đổi tọa độ thực (mét) sang tọa độ ô lưới Costmap.
* `grid2Index(mx, my)`: Đổi tọa độ 2D (x, y) sang chỉ số 1D trong mảng: `index = x + y * nx`.
* `index2Grid(i, x, y)`: Đổi chỉ số 1D ngược lại tọa độ 2D.
* `costmap_->getCharMap()[index]`: Lấy giá trị chi phí tại ô thứ `index` để kiểm tra vật cản.

### 3.2. Nhà máy khởi tạo: `PathPlannerFactory`
* **File**: `src/utils/path_planner_factory.cpp`
* Áp dụng Design Pattern **Factory**: Nhận vào chuỗi tên thuật toán (`planner_name`) từ tham số cấu hình và cấp phát đối tượng con tương ứng:
  ```cpp
  if (planner_name == "astar") {
      planner_props.planner_ptr = std::make_shared<AStarPathPlanner>(costmap_ros);
  } else if (planner_name == "dijkstra") {
      planner_props.planner_ptr = std::make_shared<AStarPathPlanner>(costmap_ros, true);
  } else if (planner_name == "jps") {
      planner_props.planner_ptr = std::make_shared<JPSPathPlanner>(costmap_ros);
  } else if (planner_name == "rrt_star") {
      planner_props.planner_ptr = std::make_shared<RRTStarPathPlanner>(costmap_ros);
  }
  ```

### 3.3. Node ROS Wrapper: `PathPlannerNode`
* **File**: `src/path_planner_node.cpp`
* Kết nối thuật toán C++ thuần với hệ sinh thái ROS:
  * Lắng nghe dịch vụ tìm đường `make_plan`.
  * Gọi `g_planner_->plan(start, goal, &path, &expand)`.
  * Publish đường đi ra topic `plan` (`nav_msgs/Path`).
  * Publish vùng mở rộng duyệt node ra topic `expand` (`nav_msgs/OccupancyGrid`) để hiển thị trên RViz.

---

## 4. CÁCH THAY ĐỔI THUẬT TOÁN KHI MUỐN CHẠY MÔ PHỎNG

### 4.1. Thay đổi trong file cấu hình
Mở file `src/user_config/user_config.yaml`:
```yaml
map: "warehouse"
world: "warehouse"
rviz_file: "sim_env_ros2.rviz"

robots_config:
  - robot1_type: "turtlebot3_waffle"
    robot1_global_planner: "astar"     # <-- ĐỔI GLOBAL PLANNER TẠI ĐÂY
    robot1_local_planner: "rpp"        # <-- ĐỔI LOCAL PLANNER TẠI ĐÂY
    robot1_x_pos: "0.0"
    robot1_y_pos: "0.0"
    robot1_z_pos: "0.0"
    robot1_yaw: "0.0"
```

### 4.2. Danh sách các Planner được hỗ trợ

Mục này được chia thành 2 phần: **Global Planner** (tìm đường toàn cục) và **Local Planner** (bám quỹ đạo và né vật cản thời gian thực).

---

#### A. Danh sách Global Planner (Khai báo tại `robot1_global_planner`)

| Phân loại | Tên khai báo | Mô tả thuật toán |
| :--- | :--- | :--- |
| **Graph-based** | `astar` | Thuật toán A* truyền thống với hàm Heuristic khoảng cách Euclid |
| | `dijkstra` | Tìm kiếm theo chiều rộng mở rộng (Heuristic = 0), đảm bảo ngắn nhất |
| | `gbfs` | Greedy Best-First Search (chỉ dùng Heuristic, tốc độ nhanh) |
| | `jps` | Jump Point Search (nhảy cóc qua ô trống, cực nhanh trên grid map) |
| | `dstar` | Dynamic A* (tái lập kế hoạch khi môi trường thay đổi) |
| | `dstar_lite` | D* Lite (phiên bản gọn gàng và hiệu quả hơn của D*) |
| | `lpa_star` | Lifelong Planning A* (cho môi trường có chướng ngại vật thay đổi dần) |
| | `voronoi` | Tìm đường theo Voronoi Diagram, giữ khoảng cách tối đa với vật cản |
| | `theta_star` | Theta* (đường đi bất kỳ góc Any-Angle, không bị gãy khúc 45/90 độ) |
| | `lazy_theta_star` | Phiên bản lười kiểm tra Line-of-sight để tăng tốc Theta* |
| | `hybrid_astar` | Tìm đường có xét bán kính quay và động học xe (xe hơi, robot có lái) |
| **Sampling-based** | `rrt` | Cây ngẫu nhiên phát triển nhanh (Rapidly-exploring Random Trees) |
| | `rrt_star` | RRT* (tối ưu hóa độ dài đường đi thông qua hàm rewire) |
| | `rrt_connect` | Phát triển đồng thời 2 cây từ Start và Goal tiến về nhau |
| | `informed_rrt` | Thu hẹp không gian lấy mẫu trong hình Elip tiêu điểm Start-Goal |
| **Evolutionary** | `aco` | Tối ưu hóa đàn kiến (Ant Colony Optimization) |
| | `pso` | Tối ưu hóa bầy hạt (Particle Swarm Optimization) |
| | `ga` | Giải thuật di truyền (Genetic Algorithm) |

---

#### B. Danh sách Local Planner / Controller (Khai báo tại `robot1_local_planner`)

Local Planner nhận đường đi toàn cục (`/plan`), kết hợp dữ liệu cảm biến từ **Local Costmap** để tính toán và phát lệnh vận tốc thực tế `cmd_vel` (`linear.x`, `angular.z`) cho robot:

| Tên khai báo | Tên đầy đủ | Nguyên lý hoạt động & Đặc điểm |
| :--- | :--- | :--- |
| `rpp` | **Regulated Pure Pursuit** | Thuật toán bám quỹ đạo hình học kinh điển; tự động điều tiết giảm vận tốc khi vào góc cua gắt hoặc khi áp sát vật cản, chạy rất mượt, ổn định và ít tốn CPU. |
| `dwa` | **Dynamic Window Approach** | Lấy mẫu không gian vận tốc khả thi (v, w) trong chu kỳ tiếp theo, mô phỏng các quỹ đạo cung tròn và chọn vận tốc tối ưu nhất theo 3 tiêu chí: bám đường, né vật cản và đạt vận tốc cao. |
| `mpc` | **Model Predictive Control** | Điều khiển dự báo mô hình; giải bài toán tối ưu hóa trong một chân trời dự đoán (predictive horizon), xét đến cả mô hình động học và giới hạn gia tốc của robot. |
| `pid` | **PID Controller** | Bộ điều khiển kinh điển (Tỉ lệ - Tích phân - Vi phân); tính toán sai số khoảng cách và sai số góc quay đến điểm mục tiêu để sinh lực kéo và quay. |
| `lqr` | **Linear Quadratic Regulator** | Bộ điều khiển tối ưu phản hồi trạng thái tuyến tính; cân bằng giữa độ chính xác bám đường và năng lượng điều khiển thông qua nghiệm của phương trình Riccati. |
| `apf` | **Artificial Potential Field** | Trường lực thế nhân tạo; coi điểm đích là nguồn hút và vật cản là nguồn đẩy, tổng hợp lực sẽ định hướng vận tốc cho robot né vật cản cục bộ. |
| `orca` | **Optimal Reciprocal Collision Avoidance** | Thuật toán né va chạm tối ưu cho hệ đa robot (Multi-agent); mỗi robot tự động gánh một phần trách nhiệm đổi hướng né nhau, tránh rơi vào bế tắc đối đầu. |
| `static` | **Static Controller** | Giữ robot đứng yên cố định (thường dùng khi chỉ cần kiểm thử thuật toán Global Planner mà không cần robot di chuyển). |

### 4.3. Các bước chạy kiểm nghiệm
1. Lưu file `user_config.yaml`.
2. Khởi chạy mô phỏng:
   ```bash
   cd ~/ros_motion_planning
   ./scripts/main.sh
   ```
3. Trên màn hình **RViz2**, click nút **2D Goal Pose** (hoặc gõ phím `g`), kéo thả chuột vào điểm đích mong muốn trên bản đồ.
4. Quan sát đường đi màu xanh (`/plan`) và vùng các node được mở rộng duyệt tìm kiếm (`/expand`).
5. Sau khi thử nghiệm xong, tắt bằng lệnh:
   ```bash
   ./scripts/killpro.sh
   ```

---

## 5. MỔ XẺ CHI TIẾT CODE THUẬT TOÁN A* (`astar_planner.cpp`)

File nguồn: `src/core/path_planner/path_planner/src/graph_planner/astar_planner.cpp`.

Đây là ví dụ điển hình nhất để hiểu cách cài đặt một Global Planner trên Costmap:

### 5.1. Định nghĩa không gian di chuyển (8 hướng)
```cpp
std::vector<AStarPathPlanner::Node> AStarPathPlanner::motions_ = {
  { 0, 1, 1.0 },           { 1, 0, 1.0 },           // Thẳng: cost = 1.0
  { 0, -1, 1.0 },          { -1, 0, 1.0 },
  { 1, 1, std::sqrt(2) },  { 1, -1, std::sqrt(2) }, // Chéo: cost = sqrt(2) ~ 1.414
  { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
};
```

### 5.2. Khởi tạo danh sách mở và đóng
* `open_list`: Sử dụng `std::priority_queue<Node, std::vector<Node>, Node::compare_cost>` sắp xếp theo tổng chi phí `f(n) = g(n) + h(n)` tăng dần.
* `closed_list`: Sử dụng `std::unordered_map<int, Node>` để lưu các node đã duyệt, tra cứu độ phức tạp trung bình `O(1)` theo `id` của node.

### 5.3. Vòng lặp chính và kiểm tra va chạm
```cpp
while (!open_list.empty()) {
  auto current = open_list.top();
  open_list.pop();

  if (closed_list.find(current.id()) != closed_list.end()) continue;
  closed_list.insert(std::make_pair(current.id(), current));
  expand->emplace_back(current.x(), current.y()); // Thu thập dữ liệu vẽ RViz

  // Nếu đã đến đích
  if (current == goal_node) {
    const auto& backtrace = _convertClosedListToPath<Node>(closed_list, start_node, goal_node);
    for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter) {
      double wx, wy;
      costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
      path->emplace_back(wx, wy); // Chuyển sang tọa độ thế giới thực
    }
    return true;
  }

  // Khám phá các nút lân cận
  for (const auto& motion : motions_) {
    auto node_new = current + motion;
    node_new.set_g(current.g() + motion.g());
    node_new.set_id(grid2Index(node_new.x(), node_new.y()));

    if (closed_list.find(node_new.id()) != closed_list.end()) continue;

    // KIỂM TRA ĐIỀU KIỆN BIÊN VÀ VẬT CẢN TRÊN COSTMAP:
    if ((node_new.id() < 0) || (node_new.id() >= map_size_) ||
        (costmap_->getCharMap()[node_new.id()] >= costmap_2d::LETHAL_OBSTACLE * config_.obstacle_inflation_factor())) {
      continue;
    }

    // TÍNH TOÁN HÀM HEURISTIC (EUCLIDEAN DISTANCE)
    if (!is_dijkstra_) {
      node_new.set_h(std::hypot(node_new.x() - goal_node.x(), node_new.y() - goal_node.y()));
    }

    // NẾU LÀ GBFS THÌ G = 0 (CHỈ DÙNG HEURISTIC)
    if (is_gbfs_) {
      node_new.set_g(0.0);
    }

    node_new.set_pid(current.id()); // Ghi nhớ nút cha để truy vết ngược
    open_list.push(node_new);
  }
}
```

> **Bài học rút ra**: 
> * Khi `is_dijkstra_ = true`: `h = 0` => `f(n) = g(n)` (Trở thành thuật toán Dijkstra).
> * Khi `is_gbfs_ = true`: `g = 0` => `f(n) = h(n)` (Trở thành thuật toán Tham lam GBFS).
> * Khi bình thường: `f(n) = g(n) + h(n)` (Trở thành A* tiêu chuẩn).

---

## 6. HƯỚNG DẪN TỰ VIẾT HOẶC THÊM MỘT THUẬT TOÁN MỚI

Khi muốn thử nghiệm một thuật toán mới (ví dụ: `my_custom_planner`):

1. **Bước 1**: Tạo file header `include/path_planner/graph_planner/my_planner.h`:
   * Kế thừa từ `public PathPlanner`.
   * Khai báo hàm `bool plan(const Point3d& start, const Point3d& goal, Points3d* path, Points3d* expand) override;`.
2. **Bước 2**: Viết logic thuật toán trong file `.cpp` tương ứng trong thư mục `src/`.
3. **Bước 3**: Mở [src/utils/path_planner_factory.cpp](file:///home/picasso/ros_motion_planning/src/core/path_planner/path_planner/src/utils/path_planner_factory.cpp):
   * Thêm `#include "path_planner/graph_planner/my_planner.h"`
   * Thêm điều kiện:
     ```cpp
     else if (planner_name == "my_planner") {
         planner_props.planner_ptr = std::make_shared<MyPlanner>(costmap_ros);
         planner_props.planner_type = GRAPH_PLANNER;
     }
     ```
4. **Bước 4**: Thêm tên thuật toán vào `CMakeLists.txt` (nếu thêm file mới ngoài các pattern có sẵn) và build lại dự án:
   ```bash
   ./scripts/build.sh
   ```
5. **Bước 5**: Điền `robot1_global_planner: "my_planner"` vào `user_config.yaml` và tận hưởng thành quả!

---

## 7. LỘ TRÌNH HỌC TẬP ĐỀ XUẤT

Để làm chủ Motion Planning từ codebase này một cách nhanh và hiệu quả nhất, hãy đi theo 4 nấc thang sau:

```
[Nấc 1: Duyệt đồ thị cơ bản]
├── astar_planner.cpp (A*, Dijkstra, GBFS)
└── jps_planner.cpp (Cách Jump Point bỏ qua các node thừa)
       ↓
[Nấc 2: Tối ưu góc & Động học thực tế]
├── theta_star_planner.cpp (Cách kiểm tra tầm nhìn Line-of-Sight để cắt góc nhọn)
└── hybrid_astar_planner.cpp (Tìm kiếm trong không gian x, y, yaw có xét bán kính bẻ lái Reeds-Shepp)
       ↓
[Nấc 3: Lấy mẫu ngẫu nhiên (Sampling-based)]
├── rrt_planner.cpp (Cách sinh điểm ngẫu nhiên sample() và mở rộng cây)
├── rrt_star_planner.cpp (Hàm rewire() nối lại các nhánh để tối ưu chi phí)
└── informed_rrt_star_planner.cpp (Lấy mẫu bên trong Ellipsoid thu hẹp)
       ↓
[Nấc 4: Chuyển tiếp sang Local Planner (Controller)]
└── Đọc thư mục src/core/controller (DWA, RPP, MPC, Pure Pursuit) để xem robot bám theo Global Path như thế nào
```
