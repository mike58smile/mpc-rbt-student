#include "../include/Planning.hpp"
#include <queue>
#include <cmath>
#include <algorithm>

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map
        // add code here
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");
        
        // Service for path
        // add code here
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
            "/plan_path",
            std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Publisher for path
        // add code here
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

        RCLCPP_INFO(get_logger(), "Planning node started.");

        // Connect to map server
        // add code here
        while (!map_client_->wait_for_service(1s)) {
            RCLCPP_INFO(get_logger(), "Waiting for map service...");
        }
        // Request map
        // add code here
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        auto future = map_client_->async_send_request(
            request,
            std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    auto response = future.get();
    if (response) {
        map_ = response->map;
        RCLCPP_INFO(this->get_logger(), "Map points to: %f, %f", map_.info.origin.position.x, map_.info.origin.position.y);
        RCLCPP_INFO(this->get_logger(), "Map resolution: %f", map_.info.resolution);
        RCLCPP_INFO(this->get_logger(), "Map received! Size: %d x %d", map_.info.width, map_.info.height);
        dilateMap();
        RCLCPP_INFO(this->get_logger(), "Map dilated!");
        //map resolution
        RCLCPP_INFO(this->get_logger(), "Map resolution: %f", map_.info.resolution);
        RCLCPP_INFO(this->get_logger(), "Map size: %d x %d", map_.info.width, map_.info.height);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive map.");
    }
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {

    aStar(request->start, request->goal);
    smoothPath();
    response->plan = path_;
    path_pub_->publish(path_);
    RCLCPP_INFO(this->get_logger(), "Callback planning.");
}

void PlanningNode::dilateMap() {
    // Nastavte poloměr rozšíření překážek (v buňkách)
    int radius = 9; // např. 2 buňky = 2 * resolution metrů

    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    int width = map_.info.width;
    int height = map_.info.height;

    // Pomocná kopie pro zápis změn
    std::vector<int8_t> new_data = map_.data;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (map_.data[y * width + x] == 100) { // překážka
                // Rozšiř do okolí
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            if (std::hypot(dx, dy) <= radius) {
                                new_data[ny * width + nx] = 100;
                            }
                        }
                    }
                }
            }
        }
    }
    dilatedMap.data = new_data;
    map_ = dilatedMap;
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    if (map_.info.width == 0 || map_.info.height == 0) {
        RCLCPP_ERROR(get_logger(), "Map is empty!");
        return;
    }

    // Convert world coordinates to map grid
    auto toGrid = [&](double wx, double wy, int &mx, int &my) {
        mx = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
        my = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
    };

    int sx, sy, gx, gy;
    toGrid(start.pose.position.x, start.pose.position.y, sx, sy);
    toGrid(goal.pose.position.x, goal.pose.position.y, gx, gy);

    auto idx = [&](int x, int y) { return y * map_.info.width + x; }; // row-major

    auto isFree = [&](int x, int y) {
        if (x < 0 || y < 0 || x >= (int)map_.info.width || y >= (int)map_.info.height) return false;
        int v = map_.data[idx(x, y)];
        return v == 0; // free space if 0
    };

    // Heuristic function (Euclidean distance of start cell to goal cell)
    auto heuristic = [&](int x, int y) {
        return std::hypot(gx - x, gy - y);
    };

    // 
    struct CompareCell {
        bool operator()(const std::shared_ptr<Cell>& a, const std::shared_ptr<Cell>& b) const {
            return a->f > b->f;
        }
    };

    std::priority_queue<std::shared_ptr<Cell>, std::vector<std::shared_ptr<Cell>>, CompareCell> openList;
    std::vector<std::shared_ptr<Cell>> allCells(map_.info.width * map_.info.height, nullptr);
    std::vector<bool> closedList(map_.info.width * map_.info.height, false);

    auto startCell = std::make_shared<Cell>(sx, sy);
    startCell->g = 0;
    startCell->h = heuristic(sx, sy);
    startCell->f = startCell->g + startCell->h;
    openList.push(startCell);
    allCells[idx(sx, sy)] = startCell;

    std::vector<std::pair<int, int>> directions = {{1,0}, {-1,0}, {0,1}, {0,-1}, {1,1}, {1,-1}, {-1,1}, {-1,-1}};

    bool found = false;
    std::shared_ptr<Cell> goalCell = nullptr;

    while (!openList.empty()) {
        auto current = openList.top();
        openList.pop();
        closedList[idx(current->x, current->y)] = true;
        
        if (current->x == gx && current->y == gy) {
            found = true;
            goalCell = current;
            break;
        }

        for (auto& d : directions) {
            int nx = current->x + d.first;
            int ny = current->y + d.second;
            if (!isFree(nx, ny) || closedList[idx(nx, ny)]) continue;

            // Calculate move cost
            double move_cost = (std::abs(d.first) + std::abs(d.second) == 2) ? std::sqrt(2.0) : 1.0;
            double g_new = current->g + move_cost; // double g_new = current->g + 1.0;
            double h_new = heuristic(nx, ny);
            double f_new = g_new + h_new;

            auto &neighbor = allCells[idx(nx, ny)];
            if (!neighbor || g_new < neighbor->g) {
                neighbor = std::make_shared<Cell>(nx, ny);
                neighbor->g = g_new;
                neighbor->h = h_new;
                neighbor->f = f_new;
                neighbor->parent = current;
                openList.push(neighbor);
            }
        }
    }

    path_.poses.clear();
    if (found && goalCell) {
        std::vector<geometry_msgs::msg::PoseStamped> rev_path;
        auto cell = goalCell;
        while (cell) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = map_.info.origin.position.x + cell->x * map_.info.resolution;
            pose.pose.position.y = map_.info.origin.position.y + cell->y * map_.info.resolution;
            pose.pose.orientation.w = 1.0;
            rev_path.push_back(pose);
            cell = cell->parent;
        }
        path_.poses.assign(rev_path.rbegin(), rev_path.rend());
        path_.header.frame_id = "map";
        path_.header.stamp = this->get_clock()->now();
    } else {
        RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    }
}

void PlanningNode::smoothPath() {
    if (path_.poses.size() < 3) {
        // Pokud je trasa příliš krátká, není co vyhlazovat
        return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> smoothedPath;
    smoothedPath.push_back(path_.poses.front()); // Přidej první bod (start)

    // Parametr pro vyhlazování (čím menší, tím hladší)
    double smoothingFactor = 0.3;

    for (size_t i = 1; i < path_.poses.size() - 1; ++i) {
        auto &prev = path_.poses[i - 1].pose.position;
        auto &current = path_.poses[i].pose.position;
        auto &next = path_.poses[i + 1].pose.position;

        // Vypočítej nový bod jako vážený průměr sousedních bodů
        geometry_msgs::msg::PoseStamped smoothedPose;
        smoothedPose.header = path_.poses[i].header;
        smoothedPose.pose.position.x = current.x + smoothingFactor * (prev.x + next.x - 2 * current.x);
        smoothedPose.pose.position.y = current.y + smoothingFactor * (prev.y + next.y - 2 * current.y);
        smoothedPose.pose.position.z = current.z; // Zůstává nezměněno
        smoothedPose.pose.orientation = path_.poses[i].pose.orientation; // Orientace zůstává nezměněna

        smoothedPath.push_back(smoothedPose);
    }

    smoothedPath.push_back(path_.poses.back()); // Přidej poslední bod (cíl)

    // Nahraď původní trasu vyhlazenou verzí
    path_.poses = smoothedPath;
}

Cell::Cell(int c, int r) {
    // add code here
    x = c;
    y = r;
    f = g = h = 0.0;
    parent = nullptr;
}
