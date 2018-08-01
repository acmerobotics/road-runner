package com.acmerobotics.splinelib.control

/**
 * This class stores the proportional, integral, and derivative gains used by [PIDFController].
 */
class PIDCoefficients(val kP: Double = 0.0, val kI: Double = 0.0, val kD: Double = 0.0)