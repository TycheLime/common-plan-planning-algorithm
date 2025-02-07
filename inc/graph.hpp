//
// Created by 15428 on 6/2/2025.
//

#ifndef LINKS_GRAPH_HPP
#define LINKS_GRAPH_HPP

#include <iostream>
#include <vector>
#include <utility>
#include <random>
#include <algorithm>
#include <ranges>
#include <queue>
#include <stack>
#include <unordered_set>
#include <unordered_map>
#include <functional>

#include "inline.hpp"

class Graph {
private:
    std::vector<std::vector<double>> adj;
    std::vector<std::pair<int, int>> obstacles;

    int lens;
    std::pair<int, int> start;
    std::pair<int, int> goal;
    std::vector<std::pair<int, int>> directions;

public:
    void add_edge(int x, int y, double weight);

    void init(int len);

    void set_weights();

    void set_obstacles(int x, int y);

    void display();

    bool is_obstacles(int x, int y);

    bool is_obstacles(std::pair<int, int> coordinate);

    void set_start(int x, int y);

    void set_goal(int x, int y);

    double sum_weights(std::vector<std::pair<int, int>> &paths);

    std::vector<std::pair<int, int>> bfs();

    std::vector<std::pair<int, int>> dfs();

    std::vector<std::pair<int, int>> dijkstra();

    std::vector<std::pair<int, int>> a_star();

    std::vector<std::pair<int, int>> greedy();

    std::vector<std::pair<int, int>> bidirectional_search();

    static double manhattan(std::pair<int, int> coor1, std::pair<int, int> coor2);

    bool is_valid(std::pair<int, int> &coordinate);

    std::vector<std::pair<int, int>> construct_path(std::pair<int, int> meeting_point,
                                                    std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash> &forward_parent,
                                                    std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash> &backward_parent);

    std::vector<std::pair<int, int>> rrt();

    std::pair<int, int> get_random_point();

    int get_nearest_node(std::vector<std::pair<int, int>>& tree, const std::pair<int, int> &point);

    std::pair<int, int> steer(std::pair<int, int> &from, std::pair<int, int> &to, double step_size);

    // 计算吸引力
    std::pair<int, int> compute_attraction_force(std::pair<int, int>& current,  std::pair<int, int>& goal, double ka);

    // 计算排斥力
    std::pair<int, int> compute_repulsion_force(std::pair<int, int> &current, double kr, double obstacle_radius);
    std::pair<int, int> update_position(std::pair<int, int>& current, std::pair<int, int>& attractive_force, std::pair<int, int>& repulsion_force);
    std::vector<std::pair<int, int>> potential_field_path_planning(double ka, double kr, double obstacle_radius, double step_size);

    // 初始化种群
    std::vector<std::pair<int, int>> genetic();

    std::vector<std::vector<std::pair<int, int>>> initialize_population(int pop_size, int max_size);

    // 计算适应度
    double fitness(std::vector<std::pair<int, int>>& path);

    // 选择
    std::vector<std::pair<int, int>> select(std::vector<std::vector<std::pair<int, int>>>& population, std::vector<double>& fitness_score);

    // 交叉
    std::vector<std::pair<int, int>> crossover(std::vector<std::pair<int, int>>& parent1, std::vector<std::pair<int, int>>& parent2);

    // 变异
    void mutate(std::vector<std::pair<int, int>>& path, double mutation_rate);
};



#endif //LINKS_GRAPH_HPP
