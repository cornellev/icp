/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#pragma once

#include <vector>
#include <cmath>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace icp {
    using Vector = Eigen::VectorXd;
    using Matrix = Eigen::MatrixXd;

    /** Rigid-body transformation. */
    struct RBTransform final {
        Vector translation;
        Matrix rotation;
        Matrix transform;
        // keep the translation and rotation matrix or just the final matrix?

    public:
        // where is the most apporiate place to initialize dim?
        RBTransform(int dim) {
            translation = Vector::Zero(dim);
            rotation = Matrix::Identity(dim, dim);
            transform = Matrix::Identity(dim + 1, dim + 1);
        }
        RBTransform(): RBTransform(2) {}  // Default to 2D

        RBTransform(Vector translation, Matrix rotation)
            : translation(translation), rotation(rotation) {
            int x = rotation.rows();
            transform = Matrix::Identity(x + 1, x + 1);
            transform.block(0, 0, x, x) = rotation;
            transform.block(0, x, x, 1) = translation;
        }
        RBTransform(Matrix transform): transform(transform) {
            int x = transform.rows() - 1;
            rotation = transform.block(0, 0, x, x);
            translation = transform.block(0, x, x, 1);
        }

        Vector apply_to(Vector v) const {
            return (transform * v.homogeneous()).segment(0, v.size());
        }

        RBTransform update(Eigen::Matrix4d transform) const {
            Eigen::Matrix4d new_transform = this->transform * transform;
            return RBTransform(new_transform);
        }

        RBTransform and_then(const RBTransform& next) const {
            return RBTransform(next.rotation * this->translation + next.translation,
                next.rotation * this->rotation);
        }

        RBTransform inverse() const {
            auto transpose = this->rotation.transpose();
            return RBTransform(-transpose * this->translation, transpose);
        }

        std::string to_string() const {
            std::stringstream stream;
            stream << "RBTransform {\n";
            stream << "  translation:\n" << translation << '\n';
            stream << "  rotation:\n" << rotation << '\n';
            stream << "  transform:\n" << transform << '\n';
            stream << "}";
            return stream.str();
        }
    };
    // using RBTransform = Eigen::Transform<double, 3, Eigen::Affine>;
    Vector get_centroid(const std::vector<Vector>& points);
}
