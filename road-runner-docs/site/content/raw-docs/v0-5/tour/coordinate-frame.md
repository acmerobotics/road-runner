---
title: "Coordinate Frames"
weight: 5
---

# Coordinate Frames

The previous discussion of motion profiling wraps up motion control for mechanisms with one degree of freedom. The remainder of the tour will extend these ideas to 2D robot movement.

## Basics

In order to describe 2D motion, we need a consistent global coordinate frame. Within this global frame, the robot's position can be described using typical Cartesian {{< katex >}} (x,y) {{< /katex >}} coordinates. In addition to the linear position, the robot has a heading {{< katex >}} \theta {{< /katex >}} defined as the angle between the front of the robot and the global {{< katex >}} x {{< /katex >}} axis. The position and heading together constitute the robot's pose.

2D vectors and poses are built into the library and serve as primitives for many other classes.

{{< tabs "coords" >}}
{{< tab "Java" >}}
```java
Vector2d position = new Vector2d(x, y);
Pose2d pose = new Pose2d(position, theta);
```
{{</ tab >}}
{{< tab "Kotlin" >}}
```kotlin
val position = Vector2d(x, y)
val pose = Pose2d(position, theta)
```
{{</ tab >}}
{{</ tabs >}}

![](/field.png)

In addition to the global coordinate frame, there is a robot coordinate frame that moves along with the robot. Paths are more conveniently described in the global frame while robot velocities are more conveniently described in the robot frame. To accommodate this, Road Runner constantly switches between frames.

## Transformations

![](/transform.png "Transformation of a velocity vector between frames")

Consider a velocity vector {{< katex >}} \vec{v}_G {{< /katex >}} in the global frame. To find the equivalent representation {{< katex >}} \vec{v}_R {{< /katex >}}in the robot frame, we can apply a rotation transformation. This transformation can be represented by the following matrix:

{{< katex >}}
\begin{bmatrix}
  \cos \theta & -\sin \theta \\\\
  \sin \theta & \cos \theta
\end{bmatrix}
{{< /katex >}}

{{< hint info >}}
If you haven't seen matrices and vectors before, check out 3Blue1Brown's series [Essence of linear algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab). The first four videos are the most relevant to this section, although I strongly recommend all of them.
{{< /hint >}}

With this matrix, the transformation is simple: {{< katex >}} \vec{v}_G = R(\theta) \, \vec{v}_R {{< /katex >}}. Since the inverse of {{< katex >}} R(\theta) {{< /katex >}} is just {{< katex >}} R(-\theta) {{< /katex >}}, {{< katex >}} \vec{v}_R = R(-\theta) \, \vec{v}_G {{< /katex >}}. Note the angular velocity {{< katex >}} \omega {{< /katex >}} remains the same between frames.

As you might expect, rotations are built into the `Vector2d` class:

{{< tabs "rots" >}}
{{< tab "Java" >}}
```java
Vector2d v = new Vector2d(x, y);
Vector2d w = v.rotated(rotAngleRad);
```
{{</ tab >}}
{{< tab "Kotlin" >}}
```kotlin
val v = Vector2d(x, y)
val w = v.rotated(rotAngleRad)
```
{{</ tab >}}
{{</ tabs >}}

More sophisticated transformations can be found in `Kinematics`. For instance, the aforementioned pose-to-pose velocity transformation is `Kinematics.fieldToRobotVelocity()`.

