package com.acmerobotics.splinelib.path

import com.acmerobotics.splinelib.Vector2d

class LineSegment(private val start: Vector2d, private val end: Vector2d) : ParametricCurve() {
    private val diff = end - start

    override fun length() = diff.norm()

    override fun internalGet(t: Double) = start + diff * t

    override fun internalDeriv(t: Double) = diff

    override fun internalSecondDeriv(t: Double) = Vector2d(0.0, 0.0)

    override fun internalThirdDeriv(t: Double) = Vector2d(0.0, 0.0)

    override fun displacementToParameter(displacement: Double) = displacement / length()

    override fun parameterDeriv(t: Double) = 1.0 / length()

    override fun parameterSecondDeriv(t: Double) = 0.0

    override fun parameterThirdDeriv(t: Double) = 0.0

    override fun toString() = "(${start.x}+${diff.x}*t,${start.y}+${diff.y}*t)"
}