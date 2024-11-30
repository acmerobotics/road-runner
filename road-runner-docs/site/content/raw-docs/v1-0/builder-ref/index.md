---
title: "Builder Reference"
weight: 100 
---

# Builder Reference

## Path Primitives

The begin pose is the origin with a heading of {{< katex >}} \frac{\pi}{6} {{< /katex >}}.

### `lineToX()`

```java
.lineToX(48)
```

{{< video lineToX >}}

### `lineToY()`

```java
.lineToY(36)
```

{{< video lineToY >}}

### `splineTo()`

```java
.splineTo(new Vector2d(48, 48), Math.PI / 2)
```

{{< video splineTo >}}

## Heading Primitives

The begin pose is the origin with a heading of {{< katex >}} \frac{\pi}{2} {{< /katex >}}.

### Tangent Heading (default)

```java
.setTangent(0)
.splineTo(new Vector2d(48, 48), Math.PI / 2)
```

{{< video tangentHeading >}}

### Constant Heading

```java
.setTangent(0)
.splineToConstantHeading(Vector2d(48, 48), Math.PI / 2)
```

{{< video constantHeading >}}

### Linear Heading

```java
.setTangent(0)
.splineToLinearHeading(Pose2d(48, 48, 0), Math.PI / 2)
```

{{< video linearHeading >}}

### Spline Heading

```java
.setTangent(0)
.splineToSplineHeading(Pose2d(48, 48, 0), Math.PI / 2)
```

{{< video splineHeading >}}
