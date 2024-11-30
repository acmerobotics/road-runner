---
title: FAQ
weight: 3
---

# FAQ

- So the straight test and turn test work great but the spline test makes everything break. What's going on?

    First off, determine whether the localization is off or the controller is sloppy. If the localizer is off, consider changing to something like tracking wheels or reign in the constraints (at least reduce accel/angular velocity). If the localizer is roughly on target, you can try adding feedback (and also probably reducing the constraints).

- How do I integrate "odometry" with RR?

    Read [this page](../../tour/kinematics).

- Can I use Road Runner without "odometry"?

    Yes, that's the default.

- How do I change units?

    Inches are strongly recommended. If you really want to use other units, write your own adapters/wrappers for the Road Runner interfaces. Many important defaults are set with inches in mind, and it's difficult to change some from the high-level API.

- How do I optimize read times?

    Read times are minimized by the use of automatic bulk reads implemented in SDK v5.4 and later. Keep in mind the quickstart enables these optimizations for all code.
