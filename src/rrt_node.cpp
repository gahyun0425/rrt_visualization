#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <sstream>
#include <cmath>
#include <random>
#include <limits>

// 3D 공간에서 점을 나타내는 구조체
struct Point {
    double x, y, z;
    Point(double x_=0.0, double y_=0.0, double z_=0.0) : x(x_), y(y_), z(z_) {}
};

// 장애물 충돌 감지 함수
bool isCollision(const Point& point, const std::vector<Point>& obstacles, double obstacle_radius, double cylinder_height) {
    for (const auto& obs : obstacles) {
        double horizontal_distance = std::sqrt(std::pow(point.x - obs.x, 2) + std::pow(point.y - obs.y, 2));
        if (horizontal_distance < obstacle_radius && point.z >= 0 && point.z <= cylinder_height) {
            return true;
        }
    }
    return false;
}

// 장애물 목록 초기화 함수
std::vector<Point> initializeObstacles() {
    return { Point(-0.8, 0, 0), Point(0, -0.8, 0), Point(0.8, 0, 0), Point(0, 0.8, 0) };
}

// 랜덤한 점을 생성하는 함수
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis_x(-1.0, 1.0);
std::uniform_real_distribution<> dis_y(-1.0, 1.0);
std::uniform_real_distribution<> dis_z(0.0, 1.0);

Point getRandomPoint() {
    return Point(dis_x(gen), dis_y(gen), dis_z(gen));
}

// 가장 가까운 노드를 찾는 함수
int findNearestNode(const std::vector<Point>& path, const Point& random_point) {
    int nearest_index = -1;
    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); ++i) {
        double distance = std::sqrt(std::pow(path[i].x - random_point.x, 2) +
                                    std::pow(path[i].y - random_point.y, 2) +
                                    std::pow(path[i].z - random_point.z, 2));
        if (distance < min_distance) {
            min_distance = distance;
            nearest_index = i;
        }
    }
    return nearest_index;
}

// 목표를 향해 경로 확장
bool extendRRT(std::vector<Point>& path, const Point& goal, const std::vector<Point>& obstacles, 
               double obstacle_radius, double cylinder_height, double step_size = 0.05) {
    const int max_iterations = 1000;
    RCLCPP_INFO(rclcpp::get_logger("rrt_visualizer"), "extendRRT 함수가 호출되었습니다. 최대 반복 횟수: %d", max_iterations);
    
    for (int i = 0; i < max_iterations; ++i) {
        Point random_point;

        // 90% 확률로 목표를 향해, 10% 확률로 무작위로 확장
        if (i % 10 != 0) {
            random_point = goal;  // 목표 방향으로 확장
            RCLCPP_INFO(rclcpp::get_logger("rrt_visualizer"), "[반복 %d] 목표 방향으로 확장 시도...", i + 1);
        } else {
            random_point = getRandomPoint();  // 무작위 확장
            RCLCPP_INFO(rclcpp::get_logger("rrt_visualizer"), "[반복 %d] 무작위 점 생성: (%.2f, %.2f, %.2f)", i + 1, random_point.x, random_point.y, random_point.z);
        }

        int nearest_index = findNearestNode(path, random_point);
        Point nearest_point = path[nearest_index];
        double direction_x = random_point.x - nearest_point.x;
        double direction_y = random_point.y - nearest_point.y;
        double direction_z = random_point.z - nearest_point.z;
        double length = std::sqrt(direction_x * direction_x + direction_y * direction_y + direction_z * direction_z);

        if (length == 0) {
            RCLCPP_WARN(rclcpp::get_logger("rrt_visualizer"), "[반복 %d] 방향 벡터 길이가 0이어서 건너뜀.", i + 1);
            continue;
        }

        // 새로운 점을 생성
        Point new_point(nearest_point.x + (direction_x / length) * step_size,
                        nearest_point.y + (direction_y / length) * step_size,
                        nearest_point.z + (direction_z / length) * step_size);

        RCLCPP_INFO(rclcpp::get_logger("rrt_visualizer"), "새로운 점 생성: (%.2f, %.2f, %.2f)", new_point.x, new_point.y, new_point.z);

        if (!isCollision(new_point, obstacles, obstacle_radius, cylinder_height)) {
            path.push_back(new_point);
            RCLCPP_INFO(rclcpp::get_logger("rrt_visualizer"), "노드 추가: (%.2f, %.2f, %.2f)", new_point.x, new_point.y, new_point.z);

            if (std::sqrt(std::pow(new_point.x - goal.x, 2) +
                          std::pow(new_point.y - goal.y, 2) +
                          std::pow(new_point.z - goal.z, 2)) < step_size) {
                path.push_back(goal);
                RCLCPP_INFO(rclcpp::get_logger("rrt_visualizer"), "목표 지점 도달: (%.2f, %.2f, %.2f)", goal.x, goal.y, goal.z);
                return true;
            }
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rrt_visualizer"), "충돌 발생: (%.2f, %.2f, %.2f)", new_point.x, new_point.y, new_point.z);
        }
    }
    RCLCPP_ERROR(rclcpp::get_logger("rrt_visualizer"), "최대 반복 횟수 도달, 경로 생성 실패");
    return false;
}

// 마커로 RRT 경로를 시각화하는 함수
void visualizePath(const std::vector<Point>& path, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub) {
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < path.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "rrt_path";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = path[i].x;
        marker.pose.position.y = path[i].y;
        marker.pose.position.z = path[i].z;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        if (i == 0 || i == path.size() - 1) {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        } else {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        }

        marker.lifetime = rclcpp::Duration::from_seconds(0);
        marker_array.markers.push_back(marker);
    }
    marker_pub->publish(marker_array);
}

// 네 개의 원기둥 마커를 시각화하는 함수
void visualizeCylinders(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub) {
    visualization_msgs::msg::MarkerArray marker_array;

    std::vector<Point> cylinder_positions = initializeObstacles();

    for (size_t i = 0; i < cylinder_positions.size(); ++i) {
        visualization_msgs::msg::Marker cylinder_marker;
        cylinder_marker.header.frame_id = "world";
        cylinder_marker.header.stamp = rclcpp::Clock().now();
        cylinder_marker.ns = "cylinder_markers";
        cylinder_marker.id = i;

        cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        cylinder_marker.action = visualization_msgs::msg::Marker::ADD;

        cylinder_marker.pose.position.x = cylinder_positions[i].x;
        cylinder_marker.pose.position.y = cylinder_positions[i].y;
        cylinder_marker.pose.position.z = 0.5;

        cylinder_marker.scale.x = 0.1;
        cylinder_marker.scale.y = 0.1;
        cylinder_marker.scale.z = 1.0;

        cylinder_marker.color.r = 0.0;
        cylinder_marker.color.g = 0.0;
        cylinder_marker.color.b = 1.0;
        cylinder_marker.color.a = 1.0;

        cylinder_marker.lifetime = rclcpp::Duration::from_seconds(0);
        marker_array.markers.push_back(cylinder_marker);
    }

    marker_pub->publish(marker_array);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("rrt_visualizer");
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

    double goal_x, goal_y, goal_z;
    std::string input_line;
    std::getline(std::cin, input_line);
    std::istringstream iss(input_line);
    iss >> goal_x >> goal_y >> goal_z;

    if (iss.fail()) {
        RCLCPP_ERROR(node->get_logger(), "잘못된 입력입니다. 숫자 3개를 공백으로 구분하여 입력하세요.");
        rclcpp::shutdown();
        return 1;
    }

    std::vector<Point> obstacles = initializeObstacles();
    double obstacle_radius = 0.1;
    double cylinder_height = 1.0;

    Point goal_point(goal_x, goal_y, goal_z);
    if (isCollision(goal_point, obstacles, obstacle_radius, cylinder_height)) {
        RCLCPP_ERROR(node->get_logger(), "목표 지점이 장애물 범위 내에 있습니다. 다른 좌표를 입력하세요.");
        rclcpp::shutdown();
        return 1;
    }

    std::vector<Point> path = { Point(0.8, -0.5, 0.5) };

    if (extendRRT(path, goal_point, obstacles, obstacle_radius, cylinder_height)) {
        RCLCPP_INFO(node->get_logger(), "경로가 성공적으로 생성되었습니다.");

        RCLCPP_INFO(node->get_logger(), "경로 노드 좌표:");
        for (size_t i = 0; i < path.size(); ++i) {
            RCLCPP_INFO(node->get_logger(), "노드 %ld: (%.2f, %.2f, %.2f)", i, path[i].x, path[i].y, path[i].z);
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "목표 지점까지 경로를 찾지 못했습니다.");
    }

    rclcpp::Rate loop_rate(1);
    while (rclcpp::ok()) {
        visualizePath(path, marker_pub);
        visualizeCylinders(marker_pub);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
