//
// Created by 15428 on 6/2/2025.
//
#include "graph.hpp"
#include "inline.hpp"

void Graph::add_edge(int x, int y, double weight) {
    if(x >= 0 && x < lens && y >=0 && y < lens){
        adj[x][y] = weight;
    }
}

void Graph::init(int len) {
    lens = len;
    adj = std::vector<std::vector<double>> (len, std::vector<double>(len, 0.0));
    directions = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1},
            {-1, 1}, {1, 1}, {-1, -1}, {-1, 1}};

    obstacles = {
            {1, 2}, {2, 2}, {3, 2}, {4, 2}, {5, 2},
            {6, 2}, {6, 3}, {6, 4}, {6, 5}, {4, 4},
            {4, 5}, {2, 4}, {2, 5}, {2, 6}, {3, 7},
            {4, 7}, {5, 7}, {6, 7}
    };
}

void Graph::set_weights() {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<> weightDist(5.0, 2.0); // 平均权重5，标准差2

    for(int i = 0; i < lens; i++){
        for (int j = 0; j < lens; ++j) {
            double digit = weightDist(gen);
            if(digit > 0) {
                add_edge(i, j, weightDist(gen));
            }
        }
    }

    for(auto& coordinate: obstacles){
        set_obstacles(coordinate.first, coordinate.second);
    }
}

void Graph::set_obstacles(int x, int y) {
    if (x >= 0 && x < adj.size() && y >= 0 && y < adj.size()) {
        adj[x][y] = -1; // -1不可达
    }
}


void Graph::display() {
    std::cout << "graph details" << std::endl;

    std::cout << "[";
    for(int i = 0; i < lens; i++){
        std::cout << "[";
        for(int j = 0; j < lens; j++){
            // Check if the current edge (i, j) is not an obstacle
            if(is_obstacles(i, j)){
                // Print infinity symbol for obstacles
                std::cout << "∞";
            } else {
                // Display the weight of the edge (i, j)
                std::cout << adj[i][j];
            }

            // Add a comma between elements except after the last element in the row
            if(j < lens - 1){
                std::cout << ", ";
            }
        }
        std::cout << "]";
        // Add a newline after each row except after the last one
        if(i < lens - 1){
            std::cout << ",\n";
        }
    }
    std::cout << "]\n";
}

bool Graph::is_obstacles(int x, int y) {
    return std::ranges::any_of(obstacles, [x, y](std::pair<int, int> obstacle){
        return x == obstacle.first && y == obstacle.second;
    });
}

void Graph::set_start(int x, int y) {
    if(x >= 0 && x < lens && y >= 0, y < lens && !is_obstacles(x, y)){
        start = {x, y};
    }else{
        std::cerr <<"error start can not set on obstacle \n";
    }
}

void Graph::set_goal(int x, int y) {
    if(x >= 0 && x < lens && y >= 0, y < lens && !is_obstacles(x, y)){
        goal = {x, y};
    } else{
        std::cerr <<"error goal can not set on obstacle \n";
    }
}

bool Graph::is_obstacles(std::pair<int, int> coordinate) {
    return is_obstacles(coordinate.first, coordinate.second);
}

double Graph::sum_weights(std::vector<std::pair<int, int>>& paths) {
    double sum = 0.0;

    for(auto& path: paths){
        sum += adj[path.first][path.second];
    }


    return sum;
}

std::vector<std::pair<int, int>> Graph::bfs() {
    std::queue<std::pair<int, int>> queue;
    std::vector<std::vector<bool>> visited (lens, std::vector<bool>(lens, false));
    std::vector<std::pair<int, int>> paths;

    if(is_obstacles(start) || is_obstacles(goal)){
        return paths;
    }

    queue.push(start);
    visited[start.first][start.second] = true;

    while(!queue.empty()){
        auto current = queue.front();
        queue.pop();
        paths.push_back(current);

        if(current == goal){
            return paths;
        }

        for(auto& dir: directions){
            std::pair<int, int> coordinate = current + dir;
            if(coordinate.first >= 0 && coordinate.second >= 0 &&
               coordinate.first < lens && coordinate.second < lens &&
               !visited[coordinate.first][coordinate.second] &&
               !is_obstacles(coordinate)){
                queue.push(coordinate);
                visited[coordinate.first][coordinate.second] = true;
            }
        }
    }

    return {};
}

std::vector<std::pair<int, int>> Graph::dfs() {
    std::stack<std::pair<int, int>> stack;
    std::vector<std::vector<bool>> visited (lens, std::vector<bool>(lens, false));
    std::vector<std::pair<int, int>> paths;

    if(is_obstacles(start) || is_obstacles(goal)){
        return paths;
    }

    stack.push(start);
    visited[start.first][start.second] = true;

    while(!stack.empty()){
        auto current = stack.top();
        stack.pop();
        paths.push_back(current);

        if(current == goal){
            return paths;
        }

        for(auto& dir: directions){
            std::pair<int, int> coordinate = current + dir;
            if(coordinate.first >= 0 && coordinate.second >= 0 &&
               coordinate.first < lens && coordinate.second < lens &&
               !visited[coordinate.first][coordinate.second] &&
               !is_obstacles(coordinate)){
                stack.push(coordinate);
                visited[coordinate.first][coordinate.second] = true;
            }
        }

    }

    return {};
}

std::vector<std::pair<int, int>> Graph::dijkstra() {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::vector<std::pair<int, int>> paths;

    if (is_obstacles(start) || is_obstacles(goal)) {
        return paths;
    }

    std::vector<std::vector<double>> dist(lens, std::vector<double>(lens, std::numeric_limits<double>::infinity()));
    std::vector<std::vector<std::pair<int, int>>> prev(lens, std::vector<std::pair<int, int>>(lens, {-1, -1}));

    dist[start.first][start.second] = 0;
    pq.push({start, 0});

    while (!pq.empty()) {
        auto [current, d] = pq.top();
        pq.pop();

        if (d > dist[current.first][current.second]) {
            continue;
        }

        if (current == goal) {
            break;
        }

        for (auto& dir : directions) {
            std::pair<int, int> coordinate = current + dir;

            if (coordinate.first >= 0 && coordinate.second >= 0 &&
                coordinate.first < lens && coordinate.second < lens &&
                !is_obstacles(coordinate)) {

                double newDist = d + adj[current.first][current.second];

                if (newDist < dist[coordinate.first][coordinate.second]) {
                    dist[coordinate.first][coordinate.second] = newDist;
                    prev[coordinate.first][coordinate.second] = current;
                    pq.push({coordinate, newDist});
                }
            }
        }
    }

    if (dist[goal.first][goal.second] != std::numeric_limits<double>::infinity()) {
        auto current = goal;
        while (current != start) {
            paths.push_back(current);
            current = prev[current.first][current.second];
        }
        paths.push_back(start);
        std::ranges::reverse(paths);
    }

    return paths;
}

std::vector<std::pair<int, int>> Graph::a_star() {
    std::priority_queue<Nodes, std::vector<Nodes>, std::greater<Nodes>> pq;
    std::vector<std::pair<int, int>> paths;

    if (is_obstacles(start) || is_obstacles(goal)) {
        return paths;
    }

    std::vector<std::vector<double>> g_dist(lens, std::vector<double>(lens, std::numeric_limits<double>::infinity()));
    std::vector<std::vector<std::pair<int, int>>> prev(lens, std::vector<std::pair<int, int>>(lens, {-1, -1}));

    g_dist[start.first][start.second] = 0;
    pq.push({start, 0, manhattan(start, goal)});

    while (!pq.empty()) {
        auto [current, g, h] = pq.top();
        pq.pop();

        if (current == goal) {
            break;
        }

        for (auto& dir : directions) {
            std::pair<int, int> coordinate = current + dir;

            if (coordinate.first >= 0 && coordinate.second >= 0 &&
                coordinate.first < lens && coordinate.second < lens &&
                !is_obstacles(coordinate)) {

                double new_g_dist = g_dist[current.first][current.second] + adj[coordinate.first][coordinate.second];
                double new_h_dist = manhattan(coordinate, goal);

                if (new_g_dist < g_dist[coordinate.first][coordinate.second]) {
                    g_dist[coordinate.first][coordinate.second] = new_g_dist;
                    prev[coordinate.first][coordinate.second] = current;
                    pq.push({coordinate, new_g_dist, new_h_dist});
                }
            }
        }
    }

    if (g_dist[goal.first][goal.second] != std::numeric_limits<double>::infinity()) {
        auto current = goal;
        while (current != start) {
            paths.push_back(current);
            current = prev[current.first][current.second];
        }
        paths.push_back(start);
        std::ranges::reverse(paths);
    }

    return paths;
}

double Graph::manhattan(std::pair<int, int> coor1, std::pair<int, int> coor2) {
    return static_cast<double>(std::abs(coor1.first - coor2.first) + std::abs(coor1.second - coor2.second));
}

std::vector<std::pair<int, int>> Graph::greedy() {
    std::priority_queue<Node, std::vector<Node>, std::greater<>> pq;
    std::vector<std::vector<bool>> visited(lens, std::vector<bool>(lens, false));
    std::vector<std::pair<int, int>> paths;

    if (is_obstacles(start) || is_obstacles(goal)) {
        return paths;
    }

    pq.push({start, manhattan(start, goal)});
    visited[start.first][start.second] = true;

    while(!pq.empty()){
        auto [current, h] = pq.top();
        pq.pop();

        paths.push_back(current);

        if(current == goal){
            return paths;
        }

        for(auto& dir: directions){
            std::pair<int, int> neighbor = current + dir;

            if (neighbor.first >= 0 && neighbor.second >= 0 &&
                neighbor.first < lens && neighbor.second < lens &&
                !visited[neighbor.first][neighbor.second] &&
                !is_obstacles(neighbor)) {
                pq.push({neighbor, manhattan(neighbor, goal)});
                visited[neighbor.first][neighbor.second] = true;
            }
        }
    }

    return {};
}

std::vector<std::pair<int, int>> Graph::bidirectional_search() {
    std::queue<std::pair<int, int>> forward_queue;
    std::queue<std::pair<int, int>> backward_queue;
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash> forward_parent;
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash> backward_parent;
    std::unordered_set<std::pair<int, int>, pair_hash> visited_forward;
    std::unordered_set<std::pair<int, int>, pair_hash> visited_backward;

    if (is_obstacles(start) || is_obstacles(goal)) {
        return {};
    }

    forward_queue.push(start);
    backward_queue.push(goal);
    visited_forward.insert(start);
    visited_backward.insert(goal);
    forward_parent[start] = {-1, -1};
    backward_parent[goal] = {-1, -1};

    while (!forward_queue.empty() && !backward_queue.empty()) {
        // 前向搜索
        if (!forward_queue.empty()) {
            auto current_forward = forward_queue.front();
            forward_queue.pop();

            for (const auto& dir : directions) {
                std::pair<int, int> next_forward = {current_forward.first + dir.first, current_forward.second + dir.second};

                if (is_valid(next_forward) && visited_forward.find(next_forward) == visited_forward.end()) {
                    forward_queue.push(next_forward);
                    visited_forward.insert(next_forward);
                    forward_parent[next_forward] = current_forward;

                    if (visited_backward.find(next_forward) != visited_backward.end()) {
                        return construct_path(next_forward, forward_parent, backward_parent);
                    }
                }
            }
        }

        // 后向搜索
        if (!backward_queue.empty()) {
            auto current_backward = backward_queue.front();
            backward_queue.pop();

            for (const auto& dir : directions) {
                std::pair<int, int> next_backward = {current_backward.first + dir.first, current_backward.second + dir.second};

                if (is_valid(next_backward) && visited_backward.find(next_backward) == visited_backward.end()) {
                    backward_queue.push(next_backward);
                    visited_backward.insert(next_backward);
                    backward_parent[next_backward] = current_backward;

                    if (visited_forward.find(next_backward) != visited_forward.end()) {
                        return construct_path(next_backward, forward_parent, backward_parent);
                    }
                }
            }
        }
    }

    return {};
}

bool Graph::is_valid(std::pair<int, int> &coordinate) {
    return coordinate.first >= 0 && coordinate.second >= 0 &&
           coordinate.first < lens && coordinate.second < lens &&
           !is_obstacles(coordinate);
}

std::vector<std::pair<int, int>> Graph::construct_path(
        std::pair<int, int> meeting_point,
        std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash>& forward_parent,
        std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash>& backward_parent) {

    std::vector<std::pair<int, int>> path;

    // 从 meeting_point 逆向构造前向路径
    std::pair<int, int> current = meeting_point;
    while (forward_parent[current] != std::make_pair(-1, -1)) {
        path.push_back(current);
        current = forward_parent[current];
    }
    path.push_back(start);  // 添加起点
    std::reverse(path.begin(), path.end()); // 逆序，得到从 start 到 meeting_point

    // 从 meeting_point 构造后向路径
    current = backward_parent[meeting_point];
    while (current != std::make_pair(-1, -1)) {
        path.push_back(current);
        current = backward_parent[current];
    }

    return path;
}


std::vector<std::pair<int, int>> Graph::rrt() {
    std::vector<std::pair<int, int>> path;
    std::vector<std::pair<int, int>> tree;
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash> parent;

    tree.push_back(start);
    parent[start] = start;
    double step_size = 1.0;
    int max_iterations = 10000;

    for (int i = 0; i < max_iterations; ++i) {
        std::pair<int, int> rand_point = get_random_point();
        int nearest_idx = get_nearest_node(tree, rand_point);
        std::pair<int, int> nearest_point = tree[nearest_idx];

        std::pair<int, int> new_point = steer(nearest_point, rand_point, step_size);

        if (is_valid(new_point)) {
            tree.push_back(new_point);
            parent[new_point] = nearest_point;

            // 检查是否接近目标点
            if (manhattan(new_point, goal) < step_size) {
                std::pair<int, int> current = new_point;
                while (current != start) {
                    path.push_back(current);
                    current = parent[current];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }
        }
    }
    return {}; // 返回空路径表示未找到
}

std::pair<int, int> Graph::steer(std::pair<int, int> &from, std::pair<int, int> &to, double step_size) {
    double theta = atan2(to.second - from.second, to.first - from.first);
    int new_x = static_cast<int>(from.first + step_size * cos(theta));
    int new_y = static_cast<int>(from.second + step_size * sin(theta));

    // 确保新点至少移动了一步
    if (new_x == from.first && new_y == from.second) {
        if (abs(cos(theta)) > abs(sin(theta))) {
            new_x += cos(theta) > 0 ? 1 : -1;
        } else {
            new_y += sin(theta) > 0 ? 1 : -1;
        }
    }

    return {new_x, new_y};
}

std::pair<int, int> Graph::get_random_point() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, lens-1);

    // 有一定概率直接朝向目标点
    if (dis(gen) % 100 < 5) { // 5%的概率
        return goal;
    }

    return {dis(gen), dis(gen)};
}

int Graph::get_nearest_node(std::vector<std::pair<int, int>>& tree, const std::pair<int, int> &point) {
    int min_index = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < tree.size(); ++i) {
        double dist = std::hypot(tree[i].first - point.first, tree[i].second - point.second);
        if (dist < min_dist) {
            min_dist = dist;
            min_index = i;
        }
    }
    return min_index;
}

std::pair<int, int>Graph::compute_attraction_force(std::pair<int, int> &current, std::pair<int, int> &goal, double ka) {
    std::pair<int, int> force = goal - current;
    force *= ka;
    return force;
}

std::pair<int, int> Graph::compute_repulsion_force(std::pair<int, int> &current, double kr, double obstacle_radius) {
    std::pair<int, int> total_force = {0, 0};
    for(auto& obstacle: obstacles){
        double dist_to_obstacle = std::sqrt(std::pow(current.first - obstacle.first, 2) +
                                            std::pow(current.second - obstacle.second, 2));
        if(dist_to_obstacle <= obstacle_radius){
            double magnitude = kr * (1.0 / (dist_to_obstacle * dist_to_obstacle) - 1.0 / (obstacle_radius * obstacle_radius));
            std::pair<int, int> direction = current - obstacle;
            total_force += direction * magnitude; // 确保 direction * magnitude 是有效的
        }
    }
    return total_force;
}

std::pair<int, int> Graph::update_position(std::pair<int, int> &current, std::pair<int, int> &attractive_force,
                                           std::pair<int, int> &repulsion_force) {
    std::pair<int, int> total_force = attractive_force + repulsion_force;

    // 找到最接近总力的方向
    std::pair<int, int> best_direction = directions[0];
    double best_dot_product = -1.0; // 初始化为负数，确保至少有一个方向会被选中
    for (const auto& direction : directions) {
        double dot_product = total_force.first * direction.first + total_force.second * direction.second;
        if (dot_product > best_dot_product) {
            best_dot_product = dot_product;
            best_direction = direction;
        }
    }

    // 使用找到的最佳方向来更新位置
    std::pair<int, int> next_position = {current.first + best_direction.first, current.second + best_direction.second};

    // 检查新位置是否有效，如果无效，则返回当前位置
    if (!is_valid(next_position)) {
        return current;
    }

    return next_position;
}


std::vector<std::pair<int, int>> Graph::potential_field_path_planning(double ka, double kr, double obstacle_radius, double step_size) {
    std::vector<std::pair<int, int>> path;
    std::pair<int, int> current = start;

    while (manhattan(current, goal) > step_size) {
        std::pair<int, int> attraction_force = compute_attraction_force(current, goal, ka);
        std::pair<int, int> repulsion_force = compute_repulsion_force(current, kr, obstacle_radius);

        std::cout << "attraction_force" << attraction_force << "\n";
        std::cout <<"repulsion_force" << repulsion_force << "\n";

        // 动态调整步长
        step_size = std::min(step_size, manhattan(current, goal) / 2.0);

        // 更新位置
        std::pair<int, int> next_position = update_position(current, attraction_force, repulsion_force);

        // 如果新位置无效，则尝试减小步长并重新计算
        if (!is_valid(next_position)) {
            step_size /= 2;
            if(step_size < 0.1) {
                std::cout << "Cannot reach the goal due to obstacles." << std::endl;
                return path;
            }
            continue;
        }

        // 重置步长
        step_size = 1.0;

        // 将当前点添加到路径中
        path.push_back(current);

        // 更新当前位置
        current = next_position;
    }

    // 添加目标点
    path.push_back(goal);

    return path;
}

std::vector<std::pair<int, int>> Graph::genetic() {
    int pop_size = 100;
    int max_steps = 30; // 最多走 30 步
    int generations = 200;
    double crossover_rate = 0.8;
    double mutation_rate = 0.05;

    // 初始化种群
    auto population = initialize_population(pop_size, max_steps);
    std::cout << population.size() << std::endl;

    for (int gen = 0; gen < generations; gen++) {
        std::vector<double> fitness_scores(pop_size);

        // 计算每条路径的适应度
        for (int i = 0; i < pop_size; i++) {
            fitness_scores[i] = fitness(population[i]);
        }

        std::vector<std::vector<std::pair<int, int>>> new_population;

        // 生成新的种群
        for (int i = 0; i < pop_size; i++) {
            auto parent1 = select(population, fitness_scores);
            auto parent2 = select(population, fitness_scores);
            auto child = crossover(parent1, parent2);
            mutate(child, mutation_rate);
            new_population.push_back(child);
        }

        population = new_population;
    }

    // 找到适应度最高的路径
    double best_fitness = -1;
    std::vector<std::pair<int, int>> best_path;
    for (auto& path : population) {
        double score = fitness(path);
        if (score > best_fitness) {
            best_fitness = score;
            best_path = path;
        }
    }

    return best_path;
}

// 初始化种群，每个个体是 max_steps 个 (dx, dy) 方向
std::vector<std::vector<std::pair<int, int>>> Graph::initialize_population(int pop_size, int max_steps) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(0, 7); // 8 个方向

    std::vector<std::vector<std::pair<int, int>>> population(pop_size);
    for (int i = 0; i < pop_size; i++) {
        std::pair<int, int> pos = start; // 路径从 start 开始
        std::vector<std::pair<int, int>> path;
        for (int j = 0; j < max_steps; j++) {
            int dir = dist(gen);
            pos += directions[dir];

            if (!is_valid(pos)) {
                break;
            }
            path.push_back(pos);
        }
        population[i] = path;
    }

    return population;
}

// 计算适应度：距离 goal 近、路径短、不碰撞障碍物
double Graph::fitness(std::vector<std::pair<int, int>>& path) {
    if (path.empty()) { // 无效路径，给极低适应度
        return -1e9;
    }

    std::pair<int, int> last = path.back();
    double dist_to_goal = manhattan(last, goal);

    // 如果到达目标点，奖励适应度
    if (last == goal) return 1e6 - path.size();

    // 路径越短越好
    return 1000.0 / (dist_to_goal + 1) - path.size();
}

// 轮盘赌选择适应度较高的个体
std::vector<std::pair<int, int>> Graph::select(std::vector<std::vector<std::pair<int, int>>>& population,
                                               std::vector<double>& fitness_scores) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<int> dist(fitness_scores.begin(), fitness_scores.end());

    return population[dist(gen)];
}

// 交叉：随机选择一个点交换后续路径
std::vector<std::pair<int, int>> Graph::crossover(std::vector<std::pair<int, int>>& parent1,
                                                  std::vector<std::pair<int, int>>& parent2) {
    if (parent1.empty() || parent2.empty()) {
        return parent1;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(0, std::min(parent1.size(), parent2.size()) - 1);

    int crossover_point = dist(gen);
    std::vector<std::pair<int, int>> child(parent1.begin(), parent1.begin() + crossover_point);
    child.insert(child.end(), parent2.begin() + crossover_point, parent2.end());

    return child;
}

// 变异：随机改变某一步的方向
void Graph::mutate(std::vector<std::pair<int, int>>& path, double mutation_rate) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> prob(0.0, 1.0);
    std::uniform_int_distribution<int> dir_dist(0, 3);

    for (auto& step : path) {
        if (prob(gen) < mutation_rate) {
            int dir = dir_dist(gen);
            step.first += directions[dir].first;
            step.second += directions[dir].second;

            if (!is_valid(step)) {
                break;
            }
        }
    }
}

//std::vector<std::pair<int, int>> Graph::genetic() {
//    int pop_size = 100;
//    int max_steps = 30;
//    int generations = 200;
//    double crossover_rate = 0.8;
//    double mutation_rate = 0.05;
//
//    auto population = initialize_population(pop_size, max_steps);
//
//    for (int gen = 0; gen < generations; gen++) {
//        std::vector<double> fitness_scores(pop_size);
//        for (int i = 0; i < pop_size; i++) {
//            fitness_scores[i] = fitness(population[i]);
//        }
//
//        std::vector<std::vector<std::pair<int, int>>> new_population;
//        for (int i = 0; i < pop_size; i++) {
//            auto parent1 = select(population, fitness_scores);
//            auto parent2 = select(population, fitness_scores);
//            auto child = crossover(parent1, parent2);
//            mutate(child, mutation_rate);
//            new_population.push_back(child);
//        }
//        population = new_population;
//    }
//
//
//    return std::vector<std::pair<int, int>>();
//}
//
//std::vector<std::vector<std::pair<int, int>>> Graph::initialize_population(int pop_size, int max_size) {
//    std::random_device rd;
//    std::mt19937 gen(rd());
//
//    std::vector<std::vector<std::pair<int, int>>> population(pop_size);
//
//    for(int i = 0; i < pop_size; i++){
//        std::vector<std::pair<int, int>> path;
//        std::pair<int, int> current = start;
//
//        for(int j = 0; j < max_size; j++){
//            if(current == goal){
//                break;
//            }
//            std::uniform_int_distribution<int> dist(0, directions.size() - 1);
//            std::pair<int, int> next = current + directions[dist(gen)];
//
//            if(is_valid(next)){
//                path.push_back(next);
//                current = next;
//            }
//        }
//        population[i] = path;
//    }
//
//    return population;
//}
//
//double Graph::fitness(std::vector<std::pair<int, int>>& path) {
//    if(path.empty()){
//        return 1e-9;
//    }
//
//    double total_distance = 0.0;
//    for(int i = 1; i < path.size(); i++){
//        total_distance += manhattan(path[i-1], path[i]);
//    }
//
//    return total_distance + manhattan(path.back(), goal);
//}
//
//std::vector<std::pair<int, int>> Graph::select(std::vector<std::vector<std::pair<int, int>>> & population, std::vector<double> & fitness_score) {
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_real_distribution<double> dist(0, 1);
//    double total_fitness = 0.0;
//    for(double f: fitness_score){
//        total_fitness += 1.0 / f;
//    }
//
//    double random = dist(gen) * total_fitness;
//    double sum = 0;
//    for(int i = 0; i < population.size(); i++){
//        sum += 1.0 / fitness_score[i];
//        if(sum > i){
//            return population[i];
//        }
//    }
//
//    return population.back();
//}
//
//std::vector<std::pair<int, int>> Graph::crossover(std::vector<std::pair<int, int>> &parent1, std::vector<std::pair<int, int>> &parent2) {
//    if (parent1.empty() || parent2.empty()){
//        return parent1;
//    }
//
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_int_distribution<int> dist(1, std::min(parent1.size(), parent2.size()) - 1);
//
//    int point = dist(gen);
//    std::vector<std::pair<int, int>> child(parent1.begin(), parent2.begin() + point);
//    child.insert(child.end(), parent2.begin() + point, parent2.end());
//    return child;
//}
//
//void Graph::mutate(std::vector<std::pair<int, int>> &path, double mutation_rate) {
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_real_distribution<double> dist(0, 1);
//    std::uniform_int_distribution<int> dir_dist(0, directions.size() - 1);
//
//    for(int i = 0; i < path.size(); i++){
//        if(dist(gen) < mutation_rate){
//            std::pair<int, int> new_pos = path[i] + directions[dir_dist(gen)];
//            if (is_valid(new_pos)) {
//                path[i] = new_pos;
//            }
//        }
//    }
//}







