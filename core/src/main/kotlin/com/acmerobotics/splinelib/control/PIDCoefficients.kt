package com.acmerobotics.splinelib.control

/**
 * This class stores the proportional, integral, and derivative gains used by [PIDFController].
 *
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 */
class PIDCoefficients(val kP: Double = 0.0, val kI: Double = 0.0, val kD: Double = 0.0)