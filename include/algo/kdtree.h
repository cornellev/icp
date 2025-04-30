/**
 * @copyright Copyright (C) 2025 Cornell Electric Vehicles.
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <vector>
#include <algorithm>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <limits>
#include <functional>

namespace icp {
    /**
     * @brief A k-d tree for efficient nearest neighbor search in k-dimensional space.
     *
     */
    template<typename PointT>
    class KdTree {
    public:
        /**
         * @brief Construct an empty k-d tree. defaults to 3D space.
         */
        KdTree(): root_(nullptr), dim_(3) {}

        /**
         * @brief Construct and build the k-d tree from a point cloud.
         * @param points Vector of input points.
         * @param dim Dimensionality of the space. Defaults to 3 if not specified.
         */
        KdTree(const std::vector<PointT>& points, int dim = -1): root_(nullptr) {
            build(points, dim);
        }

        /**
         * @brief Destructor.
         */
        ~KdTree() {
            clear();
        }

        /**
         * @brief Build the k-d tree from a given point set.
         * @param points Vector of input points.
         * @param dim Dimensionality of the space. If -1, defaults to 3.
         */
        void build(const std::vector<PointT>& points, int dim = -1) {
            clear();
            points_ = points;

            dim_ = (dim > 0) ? dim : 3;

            std::vector<int> indices(points.size());
            std::iota(std::begin(indices), std::end(indices), 0);

            root_ = build_recursive(indices.data(), static_cast<int>(points.size()), 0);
        }

        /**
         * @brief Clear the tree.
         */
        void clear() {
            clear_recursive(root_);
            root_ = nullptr;
            points_.clear();
        }

        /**
         * @brief Internal node structure of the k-d tree.
         */
        struct Node {
            int idx;        // index to the original point
            Node* next[2];  // pointers to the child nodes
            int axis;       // dimension's axis

            Node(): idx(-1), axis(-1) {
                next[0] = next[1] = nullptr;
            }
        };

        /**
         * @brief Search for the nearest neighbor to a query point.
         *
         * @param query The query point.
         * @param minDist Optional output parameter for the squared distance to the nearest point.
         * @return Index of the nearest neighbor in the original input point vector.
         */
        int search(const PointT& query, double* minDist = nullptr) const {
            int guess;
            double _minDist = std::numeric_limits<double>::max();

            search_recursive(query, root_, &guess, &_minDist);

            if (minDist) *minDist = _minDist;
            return guess;
        }

    private:
        /**
         * @brief Recursively build the tree from a set of point indices.
         */
        Node* build_recursive(int* indices, int npoints, int depth) {
            if (npoints <= 0) return nullptr;

            const int axis = depth % dim_;
            const int mid = (npoints - 1) / 2;

            std::nth_element(indices, indices + mid, indices + npoints,
                [&](int lhs, int rhs) { return points_[lhs][axis] < points_[rhs][axis]; });

            Node* node = new Node();
            node->idx = indices[mid];
            node->axis = axis;

            node->next[0] = build_recursive(indices, mid, depth + 1);
            node->next[1] = build_recursive(indices + mid + 1, npoints - mid - 1, depth + 1);

            return node;
        }

        /**
         * @brief Recursively clear the tree.
         */
        void clear_recursive(Node* node) {
            if (node == nullptr) return;

            if (node->next[0]) clear_recursive(node->next[0]);
            if (node->next[1]) clear_recursive(node->next[1]);

            delete node;
        }

        /**
         * @brief Compute squared Euclidean distance between two points.
         */
        static double distance(const PointT& p, const PointT& q) {
            double dist = 0;
            for (int i = 0; i < p.size(); i++) dist += (p[i] - q[i]) * (p[i] - q[i]);
            return std::sqrt(dist);
        }

        /**
         * @brief Recursively search for the nearest neighbor.
         */
        void search_recursive(const PointT& query, const Node* node, int* guess,
            double* minDist) const {
            if (node == nullptr) return;

            const PointT& train = points_[node->idx];
            const double dist = distance(query, train);
            if (dist < *minDist) {
                *minDist = dist;
                *guess = node->idx;
            }

            const int axis = node->axis;
            const int dir = query[axis] < train[axis] ? 0 : 1;
            search_recursive(query, node->next[dir], guess, minDist);

            const double diff = std::fabs(query[axis] - train[axis]);
            if (diff < *minDist) search_recursive(query, node->next[!dir], guess, minDist);
        }

        Node* root_;                  // root node of the k-d tree
        std::vector<PointT> points_;  // stored reference to the original input points
        int dim_;                     // dimensionality of the space
    };

}  // namespace icp
