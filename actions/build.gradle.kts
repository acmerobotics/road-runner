import java.net.URI

val libVersion: String by rootProject.extra

plugins {
    id("org.jetbrains.kotlin.jvm") version "1.9.0"

    `java-library`

    id("org.jetbrains.dokka") version "1.9.10"

    id("org.jlleitschuh.gradle.ktlint") version "10.2.1"
    id("org.jlleitschuh.gradle.ktlint-idea") version "10.2.1"

    `maven-publish`
}

repositories {
    mavenCentral()
    maven { url = URI("https://maven.brott.dev/") }
}

dependencies {
    api(kotlin("stdlib-jdk8"))

    api(project(":core"))

    api("com.acmerobotics.dashboard:core:0.4.7")

    testImplementation("org.jetbrains.kotlin:kotlin-test")

    testImplementation(testFixtures(project(":core")))
}

kotlin {
    compilerOptions {
        freeCompilerArgs.set(listOf("-Xjvm-default=all"))
    }
}

java {
    withSourcesJar()
}

tasks.named<Test>("test") {
    useJUnitPlatform()
}

publishing {
    publications {
        create<MavenPublication>("maven") {
            groupId = "com.acmerobotics.roadrunner"
            artifactId = "actions"
            version = libVersion

            from(components["java"])
        }
    }
}
