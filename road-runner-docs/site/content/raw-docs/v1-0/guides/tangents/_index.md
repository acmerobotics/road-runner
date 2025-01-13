---
title: Tangents
---

# Tangents

Picture a robot traveling along a path. The direction that the robot is moving
at any given point is the tangent at the point. Lines have a fixed tangent
direction; splines have a variable tangent.

By default, the heading of the robot follows the tangent. In fact, tank robots
are required to have the heading match the tangent (if you want to impress your
friends, this is a nonholonomic constraint). Mecanum robots are more flexible
and have a decoupled heading and tangent.

It's important to separate heading and tangent in one's mind. The tangent of
this path

```java
    .lineToX(48.0)
```

{{< video straight >}}

is the same as this path

```java
    .lineToXLinearHeading(48.0, Math.PI / 2)
```

{{< video spin >}}

despite the heading changing.

When using `TrajectoryActionBuilder`, the begin tangent of any new path segment
is chosen to match the end tangent of the last segment (this helps maintain
[continuity](../continuity)). But the tangent can still be changed manually
using `setTangent()`. (The heading cannot be changed, however, because that
would require the robot to teleport instead of merely coming to a stop.)

Finally, reversing the robot is the same as setting the tangent to be 180
degrees from where it currently is. Every call to `setReversed()` is secretly a
call to `setTangent()`.
