/**
 * @author Ethan Uppal
 * @copyright Copyright (C) 2024 Ethan Uppal.
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stddef.h>
#include <array>

template<size_t K, typename Node>
class KDTree {
    Node value;
    std::array<K, KDTree<K, Node>*> children;

public:
    KDTree();
    ~KDTree();
};
