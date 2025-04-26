# Copyright (C) 2024 Ethan Uppal.
# SPDX-License-Identifier: MIT

import sympy as sp

Point = sp.Matrix
Matrix = sp.Matrix


class RigidBodyTransform:
    def __init__(self):
        theta, tx, ty = sp.symbols("theta t_x t_y")
        self.theta = theta
        self.rotation = Matrix(
            [[sp.cos(theta), -sp.sin(theta)], [sp.sin(theta), sp.cos(theta)]]
        )
        self.translation = Point([tx, ty])

    def apply_to(self, point: Point, pivot: Point):
        return self.rotation * (point - pivot) + pivot + self.translation


class Match:
    def __init__(self):
        ax, ay, bx, by = sp.symbols("a_x a_y b_x b_y")
        self.a = Point([ax, ay])
        self.b = Point([bx, by])

    def cost_with(self, rbt: RigidBodyTransform, pivot: Point):
        v = rbt.apply_to(self.a, pivot) - self.b - pivot
        return v[0] ** 2 + v[1] ** 2


def make_centroid():
    cx, cy = sp.symbols("c_x c_y")
    return Point([cx, cy])


def latexify(f):
    return (
        str(f)
        .replace("theta", "\\theta")
        .replace("cos", "\\cos")
        .replace("sin", "\\sin")
        .replace("a_x - c_x", "a'_x")
        .replace("a_y - c_y", "a'_y")
        .replace("(a'_x)", "a'_x")
        .replace("(a'_y)", "a'_y")
        .replace("c_x", "\\bar{a}_x")
        .replace("c_y", "\\bar{a}_y")
        .replace("*", "")
    )


rbt = RigidBodyTransform()
pair = Match()
pivot = make_centroid()

cost = pair.cost_with(rbt, pivot)

print("cost function:")
print(sp.simplify(cost))
print()

print("dL/dt_x = ")
dLdt_x = sp.diff(cost, rbt.translation[0])
print(latexify(dLdt_x))
print()

(t_x_opt,) = list(sp.linsolve([dLdt_x], rbt.translation[0]))[0]
print("dL/dt_x = 0 when t_x = ")
print(latexify(t_x_opt))
print()
assert sp.simplify(dLdt_x.subs(rbt.translation[0], t_x_opt)) == 0

print("dL/dt_y = ")
dLdt_y = sp.diff(cost, rbt.translation[1])
print(latexify(dLdt_y))
print()

(t_y_opt,) = list(sp.linsolve([dLdt_y], rbt.translation[1]))[0]
print("dL/dt_y = 0 when t_y = ")
print(latexify(t_y_opt))
print()
assert sp.simplify(dLdt_y.subs(rbt.translation[1], t_y_opt)) == 0

print("dL/dtheta = ")
dLdtheta = sp.diff(cost, rbt.theta)
print(latexify(sp.simplify(dLdtheta)))
print()
