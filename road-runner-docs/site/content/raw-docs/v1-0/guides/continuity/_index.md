---
title: Continuity
---

# Continuity

The `TrajectoryActionBuilder` tries as best it can to make smooth paths without
kinks or changes of direction. Smooth paths are preferred because they can be
followed without stopping.

To make the notion of smoothness more precise, the TAB checks the continuity of
a few different values. It assumes that each individual line and spline segments
are continuous and checks that these values match at every junction.

Some violations are obvious. Take these two perpendicular line segments:

```java
    .lineToX(24.0)
    .setTangent(Math.PI / 2)
    .lineToY(24.0)
```

{{< video perp >}}

There is no way for a robot to immediately change direction like that and so it
must come to a stop at that point. This is an example of a [tangent](../tangent)
discontinuity.

Continuity is also relevant for the heading of the robot. A robot can't
instantaneously go from spinning to maintaining a fixed heading:

```java
    .lineToXLinearHeading(24.0, Math.PI / 2)
    .lineToX(48.0)
```

{{< video spinstop >}}

Similarly, a robot can't go from spinning at one speed to spinning at another
speed:

```java
    .lineToXLinearHeading(24.0, Math.PI / 2)
    .lineToXLinearHeading(48.0, 3 * Math.PI / 4)
```

Why do the two segments have different speeds? It has to do with the length of
each path segment and difference between the begin and end heading.

One way to get around this issue is to make one of them a spline heading. The
spline heading is more flexible than linear heading but has the downside of
being less preictable. You can put it at the beginning:

```java
    .lineToXSplineHeading(24.0, Math.PI / 2)
    .lineToXLinearHeading(48.0, 3 * Math.PI / 4)
```

{{< video splinespin2spin >}}

Or at the end:

```java
    .lineToXLinearHeading(24.0, Math.PI / 2)
    .lineToXSplineHeading(48.0, 3 * Math.PI / 4)
```

{{< video spin2splinespin >}}
