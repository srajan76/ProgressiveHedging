#ifndef UTIL_HPP
#define UTIL_HPP

#include <tuple>

/*  Specializing the std::hash and std::equal_to functions 
    to hash a std::tuple<int, int> and compare two std::tuple<int, int>;
    implementation for hash function flicked from boost's hash function and
    hence, do not completely understand how the hash is being performed.
    Implementation of std::equal_to<std::tuple<int, int>> is self-explanatory. 
    This is used to construct the _edgeMap in the TwoStage class. */
namespace std {
    template <> 
    struct hash<tuple<int, int>> {
        size_t operator() (const tuple<int, int>& p) const {
            size_t seed = 0;
            std::hash<int> h;
            seed ^= h(std::get<0>(p)) + 0x9e3779b9 + (seed << 6) + (seed >> 2); 
            seed ^= h(std::get<1>(p)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    template <>
    struct equal_to<tuple<int, int>> {
        bool operator() (const tuple<int, int>& x, const tuple<int, int>& y) const {
            return (get<0>(x) == get<0>(y) && get<1>(x) == get<1>(y));
        }
    };
}

#endif

