// Copyright (C) 2026 Cornell Electric Vehicles. SPDX-License-Identifier: MIT
#pragma once
#include <cstdio>
#include <cstdlib>

// Keep assertions enabled in Release so the baseline tests actually validate results.
#define assert_true(condition)                                                                     \
    do {                                                                                           \
        if (!(condition)) {                                                                        \
            std::fprintf(stderr, "%s:%d: assertion failed: %s\n", __FILE__, __LINE__, #condition); \
            std::exit(EXIT_FAILURE);                                                               \
        }                                                                                          \
    } while (false)

void test_main();
int main() {
    test_main();
    return 0;
}
