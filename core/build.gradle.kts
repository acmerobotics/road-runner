plugins {
    id("org.jetbrains.kotlin.jvm") version "1.6.20"

    `java-library`

    id("org.jetbrains.dokka") version "1.6.20"
}

repositories {
    mavenCentral()
}

dependencies {
    implementation(platform("org.jetbrains.kotlin:kotlin-bom"))

    implementation("org.jetbrains.kotlin:kotlin-stdlib-jdk8")

    implementation("org.jetbrains.kotlinx:kotlinx-collections-immutable:0.3.5")

    testImplementation("org.jetbrains.kotlin:kotlin-test")

    testImplementation("org.knowm.xchart:xchart:3.8.1")
}

java {
    withSourcesJar()
}

tasks.named<Test>("test") {
    useJUnitPlatform()
}
