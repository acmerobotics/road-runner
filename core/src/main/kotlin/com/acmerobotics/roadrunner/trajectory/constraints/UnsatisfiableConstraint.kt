package com.acmerobotics.roadrunner.trajectory.constraints

/**
 * Exception thrown when no velocity or acceleration combination exists that satisfies the constraint.
 */
abstract class UnsatisfiableConstraint : RuntimeException()