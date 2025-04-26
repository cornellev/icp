#pragma once
#include <vector>
#include <algorithm>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <limits>
#include <exception>
#include <functional>

namespace icp {

    template<typename PointT>
    class KdTree {
    public:
        KdTree(): root_(nullptr), dim_(PointT::DIM) {}
        KdTree(const std::vector<PointT>& points, int dim = -1): root_(nullptr) {
            build(points, dim);
        }
        ~KdTree() {
            clear();
        }

        void build(const std::vector<PointT>& points, int dim = -1) {
            clear();
            points_ = points;

            dim_ = (dim > 0) ? dim : 3;

            std::vector<int> indices(points.size());
            std::iota(std::begin(indices), std::end(indices), 0);

            root_ = buildRecursive(indices.data(), static_cast<int>(points.size()), 0);
        }

        void clear() {
            clearRecursive(root_);
            root_ = nullptr;
            points_.clear();
        }

        struct Node {
            int idx;        // index to the original point
            Node* next[2];  // pointers to the child nodes
            int axis;       // dimension's axis

            Node(): idx(-1), axis(-1) {
                next[0] = next[1] = nullptr;
            }
        };

        class Exception : public std::exception {
            using std::exception::exception;
        };

        int search(const PointT& query, double* minDist = nullptr) const {
            int guess;
            double _minDist = std::numeric_limits<double>::max();

            searchRecursive(query, root_, &guess, &_minDist);

            if (minDist) *minDist = _minDist;
            return guess;
        }

    private:
        Node* buildRecursive(int* indices, int npoints, int depth) {
            if (npoints <= 0) return nullptr;

            const int axis = depth % dim_;
            const int mid = (npoints - 1) / 2;

            std::nth_element(indices, indices + mid, indices + npoints,
                [&](int lhs, int rhs) { return points_[lhs][axis] < points_[rhs][axis]; });

            Node* node = new Node();
            node->idx = indices[mid];
            node->axis = axis;

            node->next[0] = buildRecursive(indices, mid, depth + 1);
            node->next[1] = buildRecursive(indices + mid + 1, npoints - mid - 1, depth + 1);

            return node;
        }

        void clearRecursive(Node* node) {
            if (node == nullptr) return;

            if (node->next[0]) clearRecursive(node->next[0]);
            if (node->next[1]) clearRecursive(node->next[1]);

            delete node;
        }

        static double distance(const PointT& p, const PointT& q) {
            double dist = 0;
            for (int i = 0; i < p.size(); i++) dist += (p[i] - q[i]) * (p[i] - q[i]);
            return std::sqrt(dist);
        }

        void searchRecursive(const PointT& query, const Node* node, int* guess,
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
            searchRecursive(query, node->next[dir], guess, minDist);

            const double diff = std::fabs(query[axis] - train[axis]);
            if (diff < *minDist) searchRecursive(query, node->next[!dir], guess, minDist);
        }

        Node* root_;
        std::vector<PointT> points_;
        int dim_;
    };

}  // namespace icp
