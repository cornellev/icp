/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#include <cassert>
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include "icp/impl/trimmed.h"
#include <unordered_set>

// TODO: should these docs be moved to the headers?

/* #name Trimmed */

/* #desc Trimmed ICP is identical to \ref vanilla_icp with the addition of an
overlap rate parameter, which specifies the percentage of points between the two
point sets that have correspondences. When the overlap rate is `1`, the algorithm
reduces to vanilla. */

//in the same mathframe work()
//paper versoin also find the nearest match from b to a 
//calculating centroid, based on dis to centroid exclude points, calculate the centroid again
namespace icp {

    Trimmed::Trimmed(double overlap_rate): ICP(), overlap_rate(overlap_rate) {}

    /* #conf "overlap_rate" A `double` between `0.0` and `1.0` for
     * the overlap rate. The default is `1.0`. */
    Trimmed::Trimmed(const Config& config)
        : ICP(), overlap_rate(config.get<double>("overlap_rate", 1)) {}

    Trimmed::~Trimmed() {}

    void Trimmed::setup() {
        a_current.resize(a.size());
        b_cm = get_centroid(b);

        for (size_t i = 0; i < a.size(); i++) {
            a_current[i] = transform.apply_to(a[i]);
        }

        compute_matches();
    }

    void Trimmed::iterate() {
        const size_t n = a.size();
        //const size_t m = b.size();

        for (size_t i = 0; i < n; i++) {
            a_current[i] = transform.apply_to(a[i]);
        }

        /* #step Matching Step: see \ref vanilla_icp for details. */
        compute_matches();

         // 如果没有足够的匹配点对，返回
        if (matches.size() < 3) {  // 需要至少3个点对来计算有意义的变换
            return;
        }

// 计算匹配点对的质心
        Vector a_cm = Vector::Zero();
        Vector b_cm = Vector::Zero();
        
        for (const auto& match : matches) {
            a_cm += a_current[match.point];
            b_cm += b[match.pair];
        }
        
        a_cm /= matches.size();
        b_cm /= matches.size();

        // 构建协方差矩阵
        Matrix N = Matrix::Zero();
        for (const auto& match : matches) {
            N += (a_current[match.point] - a_cm) * 
                 (b[match.pair] - b_cm).transpose();
        }

        // SVD分解
        auto svd = N.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        Matrix U = svd.matrixU();
        Matrix V = svd.matrixV();
        Matrix R = V * U.transpose();

        // 处理反射情况
        if (R.determinant() < 0) {
            V = V * Eigen::DiagonalMatrix<double, 2>(1, -1);
            R = V * U.transpose();
        }

        // 更新变换
        transform.rotation = R * transform.rotation;
        transform.translation = b_cm - R * a_cm;
    




        /*
            #step
            Trimming Step

            Matches are considered in increasing order of distance.

            Sources:
            https://ieeexplore.ieee.org/abstract/document/1047997
        */

       /*  std::sort(matches.begin(), matches.end(),
            [](const auto& first, const auto& second) { return first.cost < second.cost; });
        size_t new_n = static_cast<size_t>(overlap_rate * n);
        new_n = std::max<size_t>(new_n, 1);  // TODO: bad for scans with 0 points
 */
        
/*
        // a_current is A
        // b is B
        std::vector<icp::Vector> trimmed_b(a_current.size());
        for (size_t i = 0; i < trimmed_b.size(); i++) {
            trimmed_b[i] = b[matches[i].pair];
        }
        std::vector<bool> added(trimmed_b.size(), false);
        std::vector<icp::Vector> unique_b;
        for (size_t i = 0; i < trimmed_b.size(); i++) {
            if (!added[i]) {
                unique_b.push_back(trimmed_b[i]);
                added[i] = true;
            }
        }*/



        // yeah, i know this is inefficient. we'll get back to it later.
        /* std::vector<icp::Vector> trimmed_current(new_n);
        std::vector<icp::Vector> trimmed_b(new_n); */
        //std::vector<bool> added(m, false);
        /* std::unordered_set<int> b_index;
        std::vector<icp::Vector> unique_b;
        for (size_t i = 0; i < new_n; i++) {
            trimmed_current[i] = a_current[matches[i].point];
            trimmed_b[i] = b[matches[i].pair];
            if(!added[matches[i].pair]){
                unique_b.push_back(b[matches[i].pair]);
            }
            added[matches[i].pair] = true;
    
            b_index.insert(matches[i].pair);
        }
        for (const auto& index : b_index){
            unique_b.push_back(b[index]);
        } */

        /* icp::Vector trimmed_cm = get_centroid(trimmed_current); */
        //icp::Vector trimmed_b_cm1 = get_centroid(unique_b);
       /*  icp::Vector trimmed_b_cm = get_centroid(trimmed_b); */
        

        /* #step SVD: see \ref vanilla_icp for details. */
        /*
        Matrix N = Matrix::Zero();
        for (size_t i = 0; i < new_n; i++) {
            N += (trimmed_current[i] - trimmed_cm) * (trimmed_b[i] - trimmed_b_cm).transpose();
        }

        auto svd = N.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        Matrix U = svd.matrixU();
        Matrix V = svd.matrixV();
        Matrix R = V * U.transpose();

*/
        /* #step Reflection Handling: see \ref vanilla_icp for details. */
        /*
        if (R.determinant() < 0) {
            V = V * Eigen::DiagonalMatrix<double, 2>(1, -1);
            R = V * U.transpose();
        }
        

        transform.rotation = R * transform.rotation;
*/
        /* #step Transformation Step: see \ref vanilla_icp for details. */
        //transform.translation = R * transform.translation + trimmed_b_cm - R * trimmed_cm;
        //Vector translation_update = trimmed_b_cm1 - R * trimmed_cm;
        //RBTransform update_transform(translation_update, R);
        //transform = transform.compose(update_transform);
    
    }

    void Trimmed::compute_matches() {
        const size_t n = a.size();
        const size_t m = b.size();

        /*for (size_t i = 0; i < n; i++) {
            matches[i].point = i;
            matches[i].cost = std::numeric_limits<double>::infinity();
            for (size_t j = 0; j < m; j++) {
                // Point-to-point matching
                double dist_ij = (b[j] - a_current[i]).squaredNorm();

                if (dist_ij < matches[i].cost) {
                    matches[i].cost = dist_ij;
                    matches[i].pair = j;
                }
            }
        }*/
         // Forward matching (A -> B)
        std::vector<Match> forward_matches(n);
        for (size_t i = 0; i < n; i++) {
            forward_matches[i].point = i;
            forward_matches[i].cost = std::numeric_limits<double>::infinity();
            for (size_t j = 0; j < m; j++) {
                double dist_ij = (b[j] - a_current[i]).squaredNorm();
                if (dist_ij < forward_matches[i].cost) {
                    forward_matches[i].cost = dist_ij;
                    forward_matches[i].pair = j;
                }
            }
        }
        
        // Backward matching (B -> A)
        std::vector<Match> backward_matches(m);
        for (size_t j = 0; j < m; j++) {
            backward_matches[j].point = j;
            backward_matches[j].cost = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < n; i++) {
                double dist_ji = (a_current[i] - b[j]).squaredNorm();
                if (dist_ji < backward_matches[j].cost) {
                    backward_matches[j].cost = dist_ji;
                    backward_matches[j].pair = i;
                }
            }
        }
         // 只保留互为最近邻的点对
        matches.clear();
        for (size_t i = 0; i < n; i++) {
            size_t j = forward_matches[i].pair;
            if (backward_matches[j].pair == i) {
                matches.push_back(forward_matches[i]);
            }
        }

        // 根据距离对互近邻点对进行排序和trimming
        std::sort(matches.begin(), matches.end(),
            [](const auto& first, const auto& second) { return first.cost < second.cost; });
        
        // 计算需要保留的点对数量
        size_t keep_n = static_cast<size_t>(overlap_rate * std::min(matches.size(), n));
        keep_n = std::max<size_t>(keep_n, 1);  // 确保至少保留一个点对
        
        // 只保留前keep_n个最近的互近邻点对
        if (matches.size() > keep_n) {
            matches.resize(keep_n);
        }

    }
}
