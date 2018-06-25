package com.acmerobotics.library

class CompositePath(val segments: MutableList<Path> = mutableListOf()): Path(makeCompositeConstraints(segments)) {
    companion object {
        private fun makeCompositeConstraints(segments: List<Path>): MotionConstraints {
            return object : MotionConstraints {
                override fun maximumVelocity(displacement: Double): Double {
                    var remainingDisplacement = displacement
                    for (segment in segments) {
                        if (remainingDisplacement <= segment.length()) {
                            return segment.motionConstraints.maximumVelocity(remainingDisplacement)
                        }
                        remainingDisplacement -= segment.length()
                    }
                    throw RuntimeException() // TODO
                }

                override fun maximumAcceleration(displacement: Double): Double {
                    var remainingDisplacement = displacement
                    for (segment in segments) {
                        if (remainingDisplacement <= segment.length()) {
                            return segment.motionConstraints.maximumAcceleration(remainingDisplacement)
                        }
                        remainingDisplacement -= segment.length()
                    }
                    throw RuntimeException() // TODO
                }
            }
        }
    }

    override fun length() = segments.map { it.length() }.sum()

    override operator fun get(displacement: Double): Vector2d {
        var remainingDisplacement = displacement
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment[remainingDisplacement]
            }
            remainingDisplacement -= segment.length()
        }
        throw RuntimeException() // TODO
    }

    override fun deriv(displacement: Double): Vector2d {
        var remainingDisplacement = displacement
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment.deriv(remainingDisplacement)
            }
            remainingDisplacement -= segment.length()
        }
        throw RuntimeException() // TODO
    }

    override fun secondDeriv(displacement: Double): Vector2d {
        var remainingDisplacement = displacement
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment.secondDeriv(remainingDisplacement)
            }
            remainingDisplacement -= segment.length()
        }
        throw RuntimeException() // TODO
    }

    override fun thirdDeriv(displacement: Double): Vector2d {
        var remainingDisplacement = displacement
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment.thirdDeriv(remainingDisplacement)
            }
            remainingDisplacement -= segment.length()
        }
        throw RuntimeException() // TODO
    }

}