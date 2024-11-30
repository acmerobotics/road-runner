# Pose Mapping

FTC teams usually develop two versions of their autonomous routines depending on
their alliance color. The op modes are often very similar with minor changes to
reverse certain coordinates and headings. To avoid having to adjust coordinates
before giving them to `TrajectoryActionBuilder`, teams can use pose maps.

As pose map tells the builder how to transform the poses. Say you want to apply
a reflection across the x-axis: {{< katex >}} (x, y, \theta) \mapsto (x, -y,
-\theta) {{</ katex >}}. The pose map (final argument) is a pretty direct
translation:

<!-- sample: xReflection -->

When the pose map is applied to trajectory 

{{< video original >}}

the result is

{{< video mirrored >}}

Keep in mind that the pose map is applied after the trajectory and motion
profile is created. So if your constraints depend on the pose (e.g., they
specify a lower velocity for certain regions of the field), they will be
evaluated at the original, unmapped poses.
