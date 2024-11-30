---
title: Installation
weight: 1
---

# Installation

## FTC

### Quickstart

The easiest way to get started is the quickstart. The quickstart
is a full FTC Android Studio project with preinstalled Road Runner,
[FTC Dashboard](https://github.com/acmerobotics/ftc-dashboard), and tuning utilities. You
can download it from the [GitHub page](https://github.com/acmerobotics/road-runner-quickstart)
or clone it with `git clone https://github.com/acmerobotics/road-runner-quickstart.git`.
Then open the folder as an FTC project and head over to the [tuning page](../tuning) for next steps.

<!-- ## Standalone Library

1. Open `build.dependencies.gradle` and add `maven { url = 'https://maven.brott.dev/' }` to the end of the `repositories` block.
1. Open `TeamCode/build.gradle`. Add the following lines at the end of the
   `dependencies` block.

   ```groovy
   implementation 'com.acmerobotics.roadrunner:core:1.0.0'
   implementation 'com.acmerobotics.roadrunner:actions:1.0.0'
   ```

1. Sync the project \(Android Studio should prompt you to do this\). -->

### Installing into an Existing Project

{{< hint info >}}
Even if you have an existing project, it may be easier to start with the
quickstart and copy your other files over.
{{< /hint >}}

{{< hint warning >}}
If you're migrating from Road Runner 0.5.x, start by removing all references to
Road Runner in your Gradle files and elsewhere in your project. Road Runner 1.0.x
is **not** backwards compatible.
{{< /hint >}}

1. Open the `TeamCode` `build.gradle` file and add
   ```groovy
   repositories {
      maven {
         url = 'https://maven.brott.dev/'
      }
   }
   ```
   between the `android` and `dependencies` blocks. Also put
   ```groovy
   implementation "com.acmerobotics.roadrunner:ftc:0.1.14"
   implementation "com.acmerobotics.roadrunner:core:1.0.0"
   implementation "com.acmerobotics.roadrunner:actions:1.0.0"
   implementation "com.acmerobotics.dashboard:dashboard:0.4.16"
   ```
   at the end of the `dependencies` block.

1. Run a Gradle sync.
1. Download the quickstart
   [here](https://github.com/acmerobotics/road-runner-quickstart), by either
   cloning it with `git` or downloading it from GitHub.
1. Navigate to the `teamcode` folder (in the project it's at
   `TeamCode/src/main/java/org/firstinspires/ftc/teamcode`) and copy all of the
   files there (including the `messages` and `tuning` folders) to the `teamcode`
   folder of your existing project.

You're done! Time to continue on to [tuning](../tuning) or read about the new
features in Road Runner 1.0 [here](../new-features).