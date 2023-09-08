from brachiograph_step import BrachioGraphStep

# Uncomment the definition you want to use.

# This is an example BrachioGraph definition. If you build a plotter as
# described in the "Get started" section of the documentation, this definition
# is likely to work well. However, you should work out your own servo
# angle/pulse-width values as described in "Improve the plotter calibration".


# angles in degrees and corresponding pulse-widths for the two arm servos



bg = BrachioGraphStep(
    # the lengths of the arms
    inner_arm=8,
    outer_arm=8,
    virtual=True,
    turtle=True,
    turtle_coarseness=0.01,
    # the drawing area
    bounds=(-8, 4, 8, 13),
    # angles in degrees and corresponding pulse-widths for the two arm servos

    # pulse-widths for pen up/down
    pw_down=1200,
    pw_up=1850,
)


# A "naively" calibrated plotter definition. We assume the default 10ms
# pulse-width difference = 1 degree of motor movement. If the arms appear to
# move in the wrong directions, try reversing the value of servo_1_degree_ms
# and/or servo_2_degree_ms.

# naive_bg = BrachioGraph(
#     # the lengths of the arms
#     inner_arm=8,
#     outer_arm=8,
#     # the drawing area
#     bounds=(-6, 4, 6, 12),
#     # relationship between servo angles and pulse-widths
#     servo_1_degree_ms=-10,
#     servo_2_degree_ms=10,
#     # pulse-widths for pen up/down
#     pw_down=1200,
#     pw_up=1850,
# )

# Draw a box
#bg.box(bounds=[-2, 7, 2, 11])

bg.plot_file("images/demo.json")
