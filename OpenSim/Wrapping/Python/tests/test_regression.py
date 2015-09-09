"""Miscellaneous regression tests.

"""

import os

import opensim as osim


def test_property_macro_methods():
    m = osim.Model()
    g = m.get_gravity()
    assert g.get(0) == 0
    assert g.get(1) == 9.80661
    assert g.get(2) == 0

    m.upd_gravity() = osim.Vec3()

    m.set_gravity(osim.Vec3(5))

    m.set_publications("XYZ")

