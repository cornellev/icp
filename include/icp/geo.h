/*
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 */

#pragma once

#include <cassert>
#include <iostream>
#include <ostream>
#include <vector>
#include <cmath>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "Eigen/src/Core/util/Memory.h"

namespace icp {
    using Vector = Eigen::VectorXd;
    using Matrix = Eigen::MatrixXd;

    /** Rigid-body transformation. */
    struct RBTransform final {
        Matrix transform;
        Matrix rotation;
        Vector translation;
        // keep the translation and rotation matrix or just the final matrix?

    public:
        // where is the most apporiate place to initialize dim?
        RBTransform(int dim) {
            std::cout << "dim constructor called" << std::endl;
            translation = Vector::Zero(dim);
            rotation = Matrix::Identity(dim, dim);
            transform = Matrix::Identity(dim + 1, dim + 1);
        }
        ~RBTransform() {
            std::cout << "destructor" << std::endl;
            std::cout << translation.data() << std::endl;
            std::cout << rotation.data() << std::endl;
            std::cout << transform.data() << std::endl;
            std::cout << "end destructor" << std::endl;
        }
        RBTransform(): RBTransform(2) {
            std::cout << "default constructor called" << std::endl;
        }  // Default to 2D

        RBTransform(Vector translation, Matrix rotation)
            : rotation(rotation), translation(translation) {
            std::cout << "vec mat constructor" << std::endl;
            int x = rotation.rows();
            transform = Matrix::Identity(x + 1, x + 1);
            transform.block(0, 0, x, x) = rotation;
            transform.block(0, x, x, 1) = translation;
        }
        RBTransform(Matrix transform): transform(transform) {
            std::cout << "mat constructor" << std::endl;
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
