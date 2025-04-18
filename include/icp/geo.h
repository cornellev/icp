/**
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal.
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <vector>
#include <cmath>
#include <sstream>
#include <Eigen/Core>

namespace icp {
    using Vector = Eigen::Vector2d;
    using Matrix = Eigen::Matrix2d;

    /** Rigid-body transformation. */
    struct RBTransform final {
        Vector translation;
        Matrix rotation;

    public:
        RBTransform() {
            translation = Vector::Zero();
            rotation = Matrix::Identity();
        }

        RBTransform(Vector translation, Matrix rotation)
            : translation(translation), rotation(rotation) {}

        Vector apply_to(Vector v) const {
            return rotation * v + translation;
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
            stream << "}";
            return stream.str();
        }
    };

    Vector get_centroid(const std::vector<Vector>& points);
}
