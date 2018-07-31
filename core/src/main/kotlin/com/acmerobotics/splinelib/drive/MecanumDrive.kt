package com.acmerobotics.splinelib.drive

abstract class MecanumDrive(val trackWidth: Double, val wheelBase: Double = trackWidth) : Drive {
    abstract fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double)
}