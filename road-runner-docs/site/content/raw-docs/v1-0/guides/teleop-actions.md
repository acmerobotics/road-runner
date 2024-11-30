# Teleop Actions

Most sample code in the docs involving actions use `Actions.runBlocking()` to
run them. `runBlocking()` is a great fit for most autonomous programs, though
it's hard to integrate into teleop where there's already a loop monitoring the
gamepads.

Let's see what's going on inside the function and see how we can repurpose it.

<!-- sample: actionsRunBlocking -->
 
At its core, `runBlocking()` is calling `run()` on the specified action until it
returns `false`. The rest is to give feedback on the actions execution in [FTC
Dashboard](https://acmerobotics.github.io/ftc-dashboard/). We can replicate this
in teleop.

<!-- sample: actionsTeleop -->

Actions can be queued up by adding them to the list.

<!-- sample: actionsGamepadTrigger -->

Notice also how `InstantAction` saves us from writing `return false;` in the lambda.
