#pragma once
#include <vector>
#include <algorithm>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <limits>

namespace icp {

    template<typename PointT>
    class KdTree {
    public:
        struct Node {
            int axis = -1;
            float split_value = 0;
            std::unique_ptr<Node> left;
            std::unique_ptr<Node> right;
            std::vector<size_t> indices;
        };

        explicit KdTree(std::vector<PointT> points, int leaf_size = 10)
            : points_ref_(std::move(points)), leaf_size_(leaf_size) {
            if (points_ref_.empty()) {
                throw std::invalid_argument("Cannot build KdTree with empty points");
            }
            std::vector<size_t> indices(points_ref_.size());
            std::iota(indices.begin(), indices.end(), 0);
            root_ = build_tree(0, indices, 0, indices.size());
        }

        size_t find_nearest(const PointT& query, float* min_dist = nullptr) const {
            if (points_ref_.empty()) {
                if (min_dist) *min_dist = std::numeric_limits<float>::max();
                return 0;
            }

            size_t best_idx = 0;
            float best_dist = std::numeric_limits<float>::max();
            search_recursive(root_.get(), query, 0, best_idx, best_dist);
            if (min_dist) *min_dist = best_dist;
            return best_idx;
        }

    private:
        const std::vector<PointT>& points_ref_;
        const int leaf_size_;
        std::unique_ptr<Node> root_;

        std::unique_ptr<Node> build_tree(int depth, std::vector<size_t>& indices, size_t start,
            size_t end) {
            auto node = std::make_unique<Node>();
            const size_t n = end - start;

            if (n == 0 || start >= indices.size() || end > indices.size()) {
                return node;
            }

            if (n <= static_cast<size_t>(leaf_size_)) {
                node->indices.reserve(n);
                for (size_t i = start; i < end; ++i) {
                    node->indices.push_back(indices[i]);
                }
                return node;
            }

            const int dims = points_ref_[0].size();

            node->axis = depth % dims;

            try {
                auto begin = indices.begin() + start;
                auto mid = indices.begin() + start + n / 2;
                auto end_it = indices.begin() + end;

                std::nth_element(begin, mid, end_it,
                    [this, axis = node->axis](size_t a_idx, size_t b_idx) {
                        if (a_idx >= points_ref_.size() || b_idx >= points_ref_.size()) {
                            return false;
                        }
                        return points_ref_[a_idx][axis] < points_ref_[b_idx][axis];
                    });

                const size_t mid_idx = *mid;
                if (mid_idx < points_ref_.size()) {
                    node->split_value = points_ref_[mid_idx][node->axis];
                } else {
                    node->indices.assign(indices.begin() + start, indices.begin() + end);
                    return node;
                }

                node->left = build_tree(depth + 1, indices, start, start + n / 2);
                node->right = build_tree(depth + 1, indices, start + n / 2, end);
            } catch (const std::exception& e) {
                node->indices.assign(indices.begin() + start, indices.begin() + end);
            }

            return node;
        }

        void search_recursive(const Node* node, const PointT& query, int depth, size_t& best_idx,
            float& best_dist) const {
            if (!node) return;

            if (!node->indices.empty() && (node->left == nullptr && node->right == nullptr)) {
                for (size_t idx: node->indices) {
                    if (idx >= points_ref_.size()) continue;  // 安全检查
                    const float dist = (points_ref_[idx] - query).squaredNorm();
                    if (dist < best_dist) {
                        best_dist = dist;
                        best_idx = idx;
                    }
                }
                return;
            }

            const int axis = node->axis;
            if (axis < 0 || axis >= query.size()) {
                if (node->left)
                    search_recursive(node->left.get(), query, depth + 1, best_idx, best_dist);
                if (node->right)
                    search_recursive(node->right.get(), query, depth + 1, best_idx, best_dist);
                return;
            }

            const bool go_left = query[axis] < node->split_value;
            const Node* first = go_left ? node->left.get() : node->right.get();
            const Node* second = go_left ? node->right.get() : node->left.get();

            if (first) {
                search_recursive(first, query, depth + 1, best_idx, best_dist);
            }

            const float dist_to_plane = std::abs(query[axis] - node->split_value);
            if (second && dist_to_plane * dist_to_plane < best_dist) {
                search_recursive(second, query, depth + 1, best_idx, best_dist);
            }
        }
    };

}