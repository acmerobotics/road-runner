# Road Runner

A simple Kotlin library for planning 2D mobile robot paths and trajectories designed for FTC. **Note:** this project is in alpha and many of the APIs are incubating.

## Installation

### Core

1. Clone or download this repository.

1. Open up your terminal/command prompt inside the project root directory and run `./gradlew publishToMavenLocal`. This will "install" (in Maven lingo) the package on your local machine. You will need to repeat this step for each computer that you would like to build your project on.

1. Load your `ftc_app` project into Android Studio.

1. Open up the `build.gradle` file in the module you'd like to install in (usually `TeamCode`).

1. At the bottom of the file, add a `repositories` block with `jcenter()` and `mavenLocal()`. Next add a `dependencies` block with `implementation 'com.acmerobotics.roadrunner:core:0.1-SNAPSHOT'`. After you're finished, your `build.gradle` should look like this:

```groovy
// beginning of the file

repositories {
    // other repositories
    jcenter()
    mavenLocal()
}

dependencies {
    // other dependencies
    compile 'com.acmerobotics.roadrunner:core:0.1-SNAPSHOT'
}
```

1. Sync the project (Android Studio should prompt you to do this).

1. You're finished!

### GUI

Road Runner includes a simple GUI for generating trajectories from pose waypoints and constraints. Here are instructions for packaging and running it:

1. run `./gradlew shadowJar` in the the project root.

1. Navigate to `gui/build` and the executable JAR will begin with `road-runner-gui`.

### Plugin

Road Runner also includes a simple Android Studio plugin based upon the GUI. Here are some instructions for building and loading the plugin:

1. Run `./gradlew buildPlugin` in the project root.

1. In Android Studio, navigate to Settings > Plugins.

1. Click the button that reads `Install plugin from disk...`.

1. Find the Road Runner project directory and go to `plugin/build/distributions` and select the `.zip` archive.

1. Restart Android Studio to activate plugin changes.

1. Click on "Spline Designer" on the right side of the editor and the tool window should appear.

## Documentation

Run `./gradlew dokka` to generate documentation. This will make javadocs in `core/build/javadoc` and Kotlin docs in `core/build/kdoc`.