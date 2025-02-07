#include <iostream>
#include "graph.hpp"

void display_paths(const std::vector<std::pair<int, int>>& paths);

int main() {
    Graph graph;

    graph.init(10);
    graph.set_weights();
//    graph.display();
    graph.set_start(0, 3);
    graph.set_goal(7, 9);

    std::cout << "bfs\n";
    std::vector<std::pair<int, int>> paths = graph.bfs();
    display_paths(paths);
    std::cout << "sum of bfs: " << graph.sum_weights(paths) << std::endl;

    std::cout << "dfs\n";
    paths = graph.dfs();
    display_paths(paths);
    std::cout << "sum of dfs: " << graph.sum_weights(paths) << std::endl;

    std::cout << "dijkstra\n";
    paths = graph.dijkstra();
    display_paths(paths);
    std::cout << "sum of dijkstra: " << graph.sum_weights(paths) << std::endl;

    std::cout << "a*\n";
    paths = graph.a_star();
    display_paths(paths);
    std::cout << "sum of a*: " << graph.sum_weights(paths) << std::endl;

    std::cout << "greedy\n";
    paths = graph.greedy();
    display_paths(paths);
    std::cout << "sum of greedy: " << graph.sum_weights(paths) << std::endl;

    std::cout << "bidirectional_search\n";
    paths = graph.bidirectional_search();
    display_paths(paths);
    std::cout << "sum of bidirectional_search: " << graph.sum_weights(paths) << std::endl;

    std::cout << "rrt\n";
    paths = graph.rrt();
    display_paths(paths);
    std::cout << "sum of rrt: " << graph.sum_weights(paths) << std::endl;

//    std::cout << "genetic\n";
//    paths = graph.genetic();
//    display_paths(paths);
//    std::cout << "sum of genetic: " << graph.sum_weights(paths) << std::endl;

//    std::cout << "potential field\n";
//    paths = graph.potential_field_path_planning(1.3, 0.7, 1.3, 1.3);
//    display_paths(paths);
//    std::cout << "sum of potential field: " << graph.sum_weights(paths) << std::endl;

    std::cout << "Hello, World!" << std::endl;
    return 0;
}

void display_paths(const std::vector<std::pair<int, int>>& paths) {
    size_t count = 0; // 计数器，用于跟踪已处理的节点数量

    for (size_t i = 0; i < paths.size(); ++i) {
        auto path = paths[i];
        // 打印当前路径点
        std::cout << "( " << path.first << ", " << path.second << " )";

        // 如果不是最后一个点，打印箭头
        if (i < paths.size() - 1) {
            std::cout << " -> ";
        }

        // 每10个节点后换行
        ++count;
        if(count == 10 || i == paths.size() - 1) { // 当计数达到10或者到达路径末尾时
            std::cout << std::endl; // 打印换行符
            count = 0; // 重置计数器
        }
    }
}