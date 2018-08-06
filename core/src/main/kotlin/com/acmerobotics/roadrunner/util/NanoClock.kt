package com.acmerobotics.roadrunner.util

interface NanoClock {
    companion object {
        fun default() = object : NanoClock {
            override fun seconds() = System.nanoTime() / 1e9
        }
    }

    fun seconds(): Double
}