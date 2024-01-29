# """Define common units that are not SI units.
#
# Source: http://physics.nist.gov/cuu/Units/outside.html
# """
# from math import pi
#
# from units import Unum
# from units.SI import *
#
# unit = Unum.unit
#
# min = MIN = unit("min", 60 * s, "minute")
# h = H = unit("h", 60 * MIN, "hour")
# d = D = unit("d", 24 * H, "day")
# deg = ARCDEG = unit("deg", pi / 180 * RAD, "degree (angle)")
# arcmin = ARCMIN = unit("'", ARCDEG / 60, "minute (angle)")
# arcsec = ARCSEC = unit("''", ARCMIN / 60, "second (angle)")
# l = L = unit("L", 1e-3 * M**3, "liter")
# t = TON = unit("t", 1e3 * KG, "metric ton")
# Np = NP = unit("Np", 1, "neper")
# dB = DECIBEL = unit("dB", 0, "decibel")
# eV = EV = unit("eV", 1.60218e-19 * J, "electronvolt")
# u = U = unit("u", 1.66054e-27 * KG, "unified atomic mass unit")
# ua = AU = UA = unit("ua", 1.49598e11 * M, "astronomical unit")
# mile = MILE = unit("mile", 1609.34 * M, "statute mile")
# nmile = NMILE = unit("nmi", 1852 * M, "nautical mile")
# knot = KNOT = unit("knot", MILE / H, "knot")
# a = ARE = unit("a", 1e2 * M**2, "are")
# ha = HA = unit("ha", 1e4 * M**2, "hectare")
# bar = BAR = unit("bar", 1e5 * PA, "bar")
# angstrom = ANGSTROM = unit("angstrom", 1e-10 * M, "angstrom")
# b = B = unit("b", 1e-28 * M**2, "barn")
# Ci = CI = unit("Ci", 3.7e10 * BQ, "curie")
# R = R = unit("R", 2.58e-4 * C / KG, "roentgen")
# rem = REM = unit("rem", 1e-2 * SV, "rem")
#
# # Note : 'rad' defined as 1E-2 Gy as been left out because it conflits with
# # using 'rad' for radians.
#
# # cleaning
# del Unum
# del unit
# del pi
