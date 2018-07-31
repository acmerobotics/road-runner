package com.acmerobotics.splinelib.drive

abstract class TankDrive(val trackWidth: Double) : Drive {
    abstract fun setMotorPowers(left: Double, right: Double)
}