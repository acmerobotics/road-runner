---
title: FTCLib Commands
---

# FTCLib Commands

[FTCLib](https://ftclib.org/) has a [commands
system](https://docs.ftclib.org/ftclib/v/v2.0.0/command-base/command-system)
that is similar to Road Runner actions. 

Here's a generic FTCLib command for wrapping actions.

<!-- sample: actionCommand -->

This pretty much works with some caveats.
* Actions have no concept of requirements, so they need to be given for each
  command. 
* Actions do not know when they're interrupted, while commands have a chance to
  do some final cleanup. Of course, a custom command wrapping one
  particular action (e.g., following a trajectory) may specifically override
  `end()` to perform some work (e.g., stopping drive motors).
* Commands don't have a mechanism analogous to `preview()`. This means the
  quickstart trajectory preview will only work for the currently executing
  command (of course you can bundle several actions together into one composite
  action and wrap that).

