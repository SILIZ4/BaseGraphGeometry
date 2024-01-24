#ifndef BASEGRAPH_GEOMETRY_H
#define BASEGRAPH_GEOMETRY_H

#include <cmath>
#include <unordered_map>

#include "BaseGraph/algorithms/paths.hpp"
#include "BaseGraph/types.h"
#include "BaseGraph/undirected_graph.hpp"

namespace BaseGraph {
namespace geometry {


inline double distanceH2(std::vector<double> x1, std::vector<double> x2) {
    return std::acosh(std::cosh(x1[1])*std::cosh(x2[1]) - std::sinh(x1[1])*std::sinh(x2[1])*std::cos(x1[0]-x2[0]));
}

template <typename EdgeLabel>
VertexIndex
findClosestNeighbour(const LabeledUndirectedGraph<EdgeLabel> &graph,
                     VertexIndex source, VertexIndex destination,
                     const std::vector<std::vector<double>> &positions,
                     const std::function<double(const std::vector<double> &, const std::vector<double> &)> &dist) {

    double smallestDistance = algorithms::BASEGRAPH_INFINITY;
    VertexIndex closestNeighbour = graph.getSize();
    auto positionDestination = positions[destination];

    for (auto neighbour : graph.getNeighbours(source)) {
        if (neighbour != source) {
            auto distance = dist(positions[neighbour], positionDestination);
            if (distance < smallestDistance) {
                smallestDistance = distance;
                closestNeighbour = neighbour;
            }
        }
    }
    return closestNeighbour;
}

template <typename EdgeLabel>
std::unordered_map<std::string, double> getGreedyRoutingScores(
    const LabeledUndirectedGraph<EdgeLabel> &graph,
    const std::vector<std::vector<double>> &positions,
    const std::function<double(const std::vector<double> &,
                               const std::vector<double> &)> &dist) {
    std::unordered_map<std::string, double> scores;
    scores["successful"] = 0;
    scores["stretch"] = 0;
    scores["hyperstretch greedy"] = 0;
    scores["hyperstretch original"] = 0;
    auto n = graph.getSize();

    for (auto source : graph) {
        for (auto destination = source + 1; destination < n; destination++) {
            std::vector<bool> vertexExplored(n, false);
            double greedyLength = 0;
            size_t greedyHops = 0;

            VertexIndex currentVertex = source;
            while (currentVertex != destination) {
                auto closest = findClosestNeighbour(graph, currentVertex, destination, positions, dist);
                if (closest == n || vertexExplored[closest]) {
                    break;
                }
                greedyLength += dist(positions[currentVertex], positions[closest]);
                greedyHops += 1;
                vertexExplored[closest] = true;
                currentVertex = closest;
            }
            if (currentVertex == destination) {
                auto graphShortestPath = algorithms::findGeodesics(graph, source, destination);
                auto previous = n;
                double originalLength = 0;
                for (auto v: graphShortestPath) {
                    if (previous != n)
                        originalLength += dist(positions[previous], positions[v]);
                    previous = v;
                }

                double geodesicLength = dist(positions[source], positions[destination]);
                scores["successful"] += 1;
                scores["stretch"] += (double) greedyHops/(graphShortestPath.size()-1);
                scores["hyperstretch greedy"] += greedyLength/geodesicLength;
                scores["hyperstretch original"] += originalLength/geodesicLength;
            }
        }
    }
    if (scores["successful"] > 0) {
        scores["stretch"] /= scores["successful"];
        scores["hyperstretch greedy"] /= scores["successful"];
        scores["hyperstretch original"] /= scores["successful"];
        scores["successful"] = 2 * scores["successful"] / (n * (n - 1));
    }
    return scores;
}

} // namespace geometry
} // namespace BaseGraph

#endif
