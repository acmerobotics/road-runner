package com.acmerobotics.roadrunner.trajectory

fun interface DisplacementProducer {
    fun produce(length: Double): Double
}