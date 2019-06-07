# Road Runner [![Download](https://api.bintray.com/packages/acmerobotics/maven/road-runner/images/download.svg)](https://bintray.com/acmerobotics/maven/road-runner/_latestVersion) [![Trello](https://img.shields.io/badge/Vote-Trello-%2361BD4F.svg)](https://trello.com/b/Otbui84v/road-runner)

A simple Kotlin library for planning 2D mobile robot paths and trajectories designed for FTC. **Note:** this project is in alpha and many of the APIs are incubating.

<p align="center">
    <img src="doc/image/8393.gif" />
</p>
<p align="center">(Courtesy of FTC Team 8393, Detroit 2019 Ochoa F2)</p>

## Installation

### Core

1. Open your `ftc_app` project in Android Studio.

1. Open up the Gradle file for the module you'd like to install in (probably `TeamCode/build.release.gradle`).

1. Add `implementation 'com.acmerobotics.roadrunner:core:0.4.2'` to the end of the `dependencies` block.

1. (Android only) Although Road Runner only has a few dependencies, these may exceed the method reference limit imposed by some versions of Android. Fortunately, there are a few ways around this. For more information, see [this article](https://developer.android.com/studio/build/multidex).

    1. **If you do not need to target API level 19** (i.e., in FTC, you don't need to use ZTE speeds), then just add `multiDexEnabled true` to the `defaultConfig` closure (for FTC, this is located inside `build.common.gradle`).

    1. Otherwise, the next best solution is to enable Proguard (the pre Android 5.0 solution in the article is difficult to implement with the FTC SDK). To accomplish this, add the following lines to the `debug` and `release` closures inside `buildTypes` (this is also located in `build.common.gradle` for FTC):

        ```groovy
        minifyEnabled true
        proguardFiles getDefaultProguardFile('proguard-android.txt'),
                'proguard-rules.pro'
        ```

        Now download the `proguard-rules.pro` file from [the quickstart](https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/proguard-rules.pro) and save it to your module folder (`TeamCode` in the case of FTC).

    1. Finally, if the other solutions prove unworkable, you can download a slim jar from Releases. It lacks some of the features of the normal distribution, but it has fewer dependencies.

1. Sync the project (Android Studio should prompt you to do this).

1. You're finished.

### GUI

Road Runner includes a simple GUI for generating trajectories from pose waypoints and constraints. You can download the latest version from the Releases tab (or build it with `./gradlew shadowJar`).

### Plugin

Road Runner also includes a simple IDEA/Android Studio plugin based upon the GUI. Here are some instructions for building and loading the plugin:

1. Download the latest plugin zip from Releases or build it with `./gradlew buildPlugin`.

1. In Android Studio, navigate to Settings > Plugins.

1. Click the button that reads `Install plugin from disk...`.

1. Select the zip archive from earlier.

1. Restart Android Studio to activate plugin changes.

1. Click on "Path Designer" on the right side of the editor and the tool window should appear.

## Documentation

Check out our new [online documentation](https://acme-robotics.gitbook.io/road-runner/).