import java.net.URI

val libVersion: String by rootProject.extra

plugins {
    id("org.jetbrains.kotlin.jvm") version "1.6.20"

    `java-library`

    id("org.jetbrains.dokka") version "1.7.0"

    id("org.jlleitschuh.gradle.ktlint") version "10.2.1"
    id("org.jlleitschuh.gradle.ktlint-idea") version "10.2.1"

    `maven-publish`
}

repositories {
    mavenCentral()
    maven { url = URI("https://maven.brott.dev/") }
}

dependencies {
    implementation(kotlin("stdlib-jdk8"))

    implementation(project(":core"))

    implementation("com.acmerobotics.dashboard:core:0.4.7")

    testImplementation("org.jetbrains.kotlin:kotlin-test")
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
