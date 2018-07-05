package com.acmerobotics.library.path

class TangentInterpolator: HeadingInterpolator {
    private lateinit var splineSegment: QuinticSplineSegment

    override fun init(splineSegment: QuinticSplineSegment) {
        this.splineSegment = splineSegment
    }

    override fun get(displacement: Double) = splineSegment.tangentAngle(displacement)

    override fun deriv(displacement: Double) = splineSegment.tangentAngleDeriv(displacement)

    override fun secondDeriv(displacement: Double) = splineSegment.tangentAngleSecondDeriv(displacement)
}