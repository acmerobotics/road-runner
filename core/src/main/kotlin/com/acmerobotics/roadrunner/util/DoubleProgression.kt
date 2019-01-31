package com.acmerobotics.roadrunner.util

import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * A progression of values of type `Double`.
 */
class DoubleProgression(
    val start: Double,
    val endInclusive: Double,
    val step: Double
): Iterable<Double> {
    private val range: IntRange = IntRange(
        (start / step).roundToInt(),
        (endInclusive / step).roundToInt()
    )

    operator fun plus(offset: Double) =
        DoubleProgression(start + offset, endInclusive + offset, step)

    operator fun minus(offset: Double) =
        DoubleProgression(start - offset, endInclusive - offset, step)

    inner class IteratorImpl(range: IntRange) : Iterator<Double> {
        private val iterator: Iterator<Int> = range.iterator()

        override fun hasNext() = iterator.hasNext()

        override fun next() = step * iterator.next()
    }

    fun isEmpty() = range.isEmpty()

    override fun equals(other: Any?): Boolean =
            other is DoubleProgression && range == other.range && step == other.step

    override fun hashCode(): Int = range.hashCode() * 31 + step.hashCode()

    override fun iterator() = IteratorImpl(range)

    fun clip(start: Double, endInclusive: Double): DoubleProgression {
        val newStart = max(start, this.start)
        val newEnd = min(endInclusive, this.endInclusive)
        return DoubleProgression(newStart, newEnd, step)
    }
}

operator fun Double.plus(progression: DoubleProgression) = progression + this

operator fun Double.minus(progression: DoubleProgression) =
    DoubleProgression(this - progression.start, this - progression.endInclusive, progression.step)