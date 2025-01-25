/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#pragma once

#include <vector>
#include <cmath>
#include <sstream>
#include <Eigen/Core>
#include <iostream>

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
        RBTransform compose(const RBTransform& other) const {
            Eigen::Matrix2d new_rotation = this->rotation * other.rotation;
            Eigen::Vector2d new_translation = this->rotation * other.translation + this->translation;
            return RBTransform(new_translation, new_rotation);
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
