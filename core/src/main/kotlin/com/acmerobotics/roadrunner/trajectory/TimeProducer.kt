package com.acmerobotics.roadrunner.trajectory

fun interface TimeProducer {
    fun produce(duration: Double): Double
}