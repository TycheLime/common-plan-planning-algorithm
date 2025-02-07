//
// Created by 15428 on 6/2/2025.
//

#ifndef LINKS_INLINE_HPP
#define LINKS_INLINE_HPP

inline std::pair<int, int> operator + (const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) {
    return {lhs.first + rhs.first, lhs.second + rhs.second};
}

inline std::pair<int, int> operator - (const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) {
    return {lhs.first - rhs.first, lhs.second - rhs.second};
}

inline std::pair<int, int> operator * (const std::pair<int, int>& lhs, int k){
    return {k * lhs.first, k * lhs.second};
}

inline std::pair<int, int> operator * (const std::pair<int, int>& lhs, double k){
    return {static_cast<int>(k * lhs.first), static_cast<int>(k * lhs.second)};
}

inline std::pair<int, int> operator * (double k, const std::pair<int, int>& lhs){
    return {k * lhs.first, k * lhs.second};
}

inline std::pair<int, int> operator += (const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) {
    return {lhs.first + rhs.first, lhs.second + rhs.second};
}

inline std::pair<int, int> operator *= (const std::pair<int, int>& lhs, int k) {
    return {lhs.first * k, lhs.second * k};
}

inline std::ostream& operator<<(std::ostream& os, const std::pair<int, int>& p) {
    os << "(" << p.first << ", " << p.second << ")";
    return os;
}

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);
        return hash1 ^ (hash2 << 1); // XOR and bit shift
    }
};

struct Node{
    std::pair<int, int> coordinate;
    // dijkstra是距离，greedy其实是h，启发式函数
    double distance;

    bool operator > (const Node& other) const{
        return distance > other.distance;
    }
};

struct Nodes{
    std::pair<int, int> coordinate;
    double g_dist;  // g(n)
    double h_dist;  // h(n)

    bool operator > (const Nodes& other) const{
        return (g_dist + h_dist) > (other.g_dist + other.h_dist);
    }
};


#endif //LINKS_INLINE_HPP
