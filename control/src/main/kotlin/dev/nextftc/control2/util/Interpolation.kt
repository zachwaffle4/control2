/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

@file:JvmName("Interpolation")

package dev.nextftc.control2.util

import dev.nextftc.linalg.Matrix
import dev.nextftc.linalg.Nat
import dev.nextftc.units.Measure
import dev.nextftc.units.Unit
import java.util.NavigableMap
import java.util.TreeMap


/**
 * @usesMathJax
 *
 * Linearly interpolates (remaps) a value from one range to another.
 *
 * Maps [value] from the input range \([inputMin, inputMax]\) to the output range \([outputMin, outputMax]\).
 *
 * @param value The value to interpolate
 * @param inputMin The lower bound of the input range
 * @param inputMax The upper bound of the input range
 * @param outputMin The lower bound of the output range
 * @param outputMax The upper bound of the output range
 * @return The interpolated value in the output range
 */
fun lerp(value: Double, inputMin: Double, inputMax: Double, outputMin: Double, outputMax: Double) =
    if (inputMin == inputMax) 0.0
    else
        outputMin + (value - inputMin) * (outputMax - outputMin) / (inputMax - inputMin)

/**
 * @usesMathJax
 *
 * Linearly interpolates between \(low\) and \(high\) at value [t].
 *
 * @param [t] interpolation factor, typically in the range \([0, 1]\)
 */
fun lerp(low: Double, high: Double, t: Double) = low + (high - low) * t

/**
 * @usesMathJax
 *
 * Inverse of [lerp]. Given a value [value] in the range \([low, high]\), returns the interpolation factor
 * \(t\) such that \(\text{lerp}(low, high, t) = value\).
 *
 * @param [value] value in the range \([low, high]\)
 * @return interpolation factor \(t\)
 */
fun antiLerp(value: Double, low: Double, high: Double) = (value - low) / (high - low)

/**
 * Searches for the nearest value in [source] to the given [query] value.
 * If [query] exactly matches a value in [source], the corresponding value in [target] is returned.
 * If not, the two nearest values in [source] are found, and linear interpolation is performed
 * between the corresponding values in [target].
 *
 * @param [source] sorted list of source values
 * @param [target] sorted list of target values
 * @param [query] value to search for
 */
fun lerpLookup(source: List<Double>, target: List<Double>, query: Double): Double {
    require(source.size == target.size) { "source.size (${source.size}) != target.size (${target.size})" }
    require(source.isNotEmpty()) { "source is empty" }

    val index = source.binarySearch(query)
    return if (index >= 0) {
        target[index]
    } else {
        val insIndex = -(index + 1)
        when {
            insIndex <= 0 -> target.first()
            insIndex >= source.size -> target.last()
            else -> {
                val sLo = source[insIndex - 1]
                val sHi = source[insIndex]
                val tLo = target[insIndex - 1]
                val tHi = target[insIndex]
                lerp(query, inputMin = sLo, inputMax = sHi, outputMin = tLo, outputMax = tHi)
            }
        }
    }
}

/**
 * Linearly interpolates between two matrices at value [t].
 *
 * @param t interpolation factor, typically in the range \([0, 1]\)
 * @return interpolated matrix
 */
fun <R: Nat, C: Nat> lerpMatrix(t: Double, low: Matrix<R, C>, high: Matrix<R, C>) =
    low + (high - low) * t

/**
 * Linearly interpolates between two measures at value [t].
 *
 * @param t interpolation factor, typically in the range \([0, 1]\)
 * @return interpolated measure
 */
fun <U: Unit<U>> lerpMeasure(t: Double, low: Measure<U>, high: Measure<U>) =
    low + (high - low) * t

/**
 * A navigable map that supports interpolation between values.
 *
 * `InterpolatingMap` is a wrapper around a `TreeMap<Double, T>` that, when queried with a key,
 * returns the exact value if the key exists, or interpolates between the two nearest values otherwise.
 * The interpolation logic is provided by the user via the `interpolate` function.
 *
 * @param T The type of values stored in the map.
 * @property tree The underlying `TreeMap` storing the data.
 * @property interpolate A function that takes two values and an interpolation factor in [0.0, 1.0], and returns an interpolated value.
 *
 * @constructor Creates an empty `InterpolatingMap` with the given interpolation function.
 * @constructor Creates an `InterpolatingMap` with the given keys and values, using the provided interpolation function.
 *
 * @throws IllegalArgumentException if the number of keys and values do not match.
 */
class InterpolatingMap<T> private constructor(
    val tree: TreeMap<Double, T>,
    val interpolate: (T, T, Double) -> T
) : NavigableMap<Double, T> by tree {
    constructor(interpolator: (T, T, Double) -> T) : this(TreeMap(), interpolator)

    constructor(interpolator: (T, T, Double) -> T, keys: List<Double>, values: List<T>) :
            this(TreeMap(), interpolator) {
        require(keys.size == values.size) { "Keys and values must be the same size" }
        for (i in keys.indices) {
            tree[keys[i]] = values[i]
        }
    }

    /**
     * Gets the value associated with the given key.
     * If the key does not exist,
     * the value is interpolated between the two nearest values.
     */
    override fun get(key: Double): T {
        val low = floorEntry(key)
        val high = ceilingEntry(key)

        if (low.key == high.key) {
            return tree[key]!!
        }

        val t = lerp(key, inputMin = low.key, inputMax = high.key, outputMin = 0.0, outputMax = 1.0)

        return interpolate(low.value, high.value, t)
    }
}