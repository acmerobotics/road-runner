---
draft: true
---

Everyday use of Road Runner concentrates on building paths and trajectories to
follow and testing them out in autonomous routines. They are the most common
types in use and comprise the basic entities out of which op modes are made.
They behave like atomic pieces to be slotted together. But viewed from another
perspective, trajectories and paths are the apex of a careful hierarchy. They
themselves are composites constructed of layers that incrementally augment
another set of true primitives. Unmasking those levels will give a new
understanding of the library and its limitations.

Before beginning, let me warn that there is geometry, calculus, and control
theory baked into the design and implementation. Explanation of all these
concepts in depth is beyond the scope of the tutorial, though there are many
excellent resources covering these prerequisites. Here's a curated list of my
favorites:
* Khan Academy
* Astrom and Murray

(TODO: defer listing resources to the individual sections? maybe just declare
calc ab/bc req here)

In most sections, I'll write at the level of a student having completed the
differential half of an AP Calculus AB or BC course. Sections requiring more
advanced material will be explicitly noted. I'll attempt some exposition at the
high school level, though comprehensive treatment will be left to external
resources. 

# Paths

Let's begin with a close look at paths. Specifically, let's begin with the
interface `PosePath`. It has one main method, `get()` that turns a displacement
along the path into a pose. In other words, the path is a function from
displacement to pose. Like much of the library code, the `PosePath` abstraction
takes inspiration from mathematics.

The word _displacement_ here means the signed distance along the path. It's the
distance traveled by a pen tracing out the path shape with sign indicating the
direction. Otherwise identical paths with opposing directions will be traversed
in opposite ways by the robot.

The _pose_ gives the coordinates {{< katex >}} (x, y, \theta) {{< /katex >}} of the robot. The {{< katex >}} x {{< /katex >}}- and
{{< katex >}} y {{< /katex >}}-values describe the position of the robot in the plane, and the
{{< katex >}} \theta {{< /katex >}}-coordinate describes its rotation or heading. These three values are
enough to uniquely locate the robot in space.



# 
