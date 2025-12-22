/*
 * Copyright (c) 2025 NextFTC Team
 *
 *  Use of this source code is governed by an BSD-3-clause
 *  license that can be found in the LICENSE.md file at the root of this repository or at
 *  https://opensource.org/license/bsd-3-clause.
 */

@file:JvmName("Controllers")
@file:JvmMultifileClass

package dev.nextftc.control2.pipeline

import dev.nextftc.control2.feedforward.ArmFeedforward
import dev.nextftc.control2.feedforward.ElevatorFeedforward
import dev.nextftc.control2.feedforward.GravityFeedforwardParameters
import dev.nextftc.control2.feedforward.SimpleFFCoefficients
import dev.nextftc.control2.feedforward.SimpleFeedforward
import dev.nextftc.control2.model.MotionState
import dev.nextftc.units.Unit

/**
 * Creates a [Function] that wraps a [SimpleFeedforward] controller.
 *
 * The returned function takes a [MotionState] representing the desired state
 * and returns the feedforward output based on the velocity and acceleration.
 *
 * @param U the unit type for the motion state
 * @param coefficients the feedforward coefficients (kS, kV, kA)
 * @return a [Function] that computes feedforward output from a motion state
 */
fun <U : Unit<U>> simpleFeedforward(coefficients: SimpleFFCoefficients): Function<MotionState<U>, Double> {
    val ff = SimpleFeedforward<U>(coefficients)
    return Function { ff.calculate(it) }
}

/**
 * Creates a [Function] that wraps a [SimpleFeedforward] controller.
 *
 * The returned function takes a [MotionState] representing the desired state
 * and returns the feedforward output based on the velocity and acceleration.
 *
 * @param U the unit type for the motion state
 * @param kS static friction gain - minimum output to overcome static friction
 * @param kV velocity gain - output needed to maintain velocity
 * @param kA acceleration gain - output needed to accelerate (defaults to 0)
 * @return a [Function] that computes feedforward output from a motion state
 */
@JvmOverloads
fun <U : Unit<U>> simpleFeedforward(kS: Double, kV: Double, kA: Double = 0.0): Function<MotionState<U>, Double> =
    simpleFeedforward(
        SimpleFFCoefficients(kS, kV, kA),
    )

/**
 * Creates a [Function] that wraps an [ElevatorFeedforward] controller.
 *
 * The returned function takes a [MotionState] representing the desired state
 * and returns the feedforward output with constant gravity compensation.
 *
 * @param U the unit type for the motion state
 * @param coefficients the feedforward parameters (kG, kS, kV, kA)
 * @return a [Function] that computes feedforward output from a motion state
 */
fun <U : Unit<U>> elevatorFeedforward(coefficients: GravityFeedforwardParameters): Function<MotionState<U>, Double> {
    val ff = ElevatorFeedforward<U>(coefficients)
    return Function { ff.calculate(it) }
}

/**
 * Creates a [Function] that wraps an [ElevatorFeedforward] controller.
 *
 * The returned function takes a [MotionState] representing the desired state
 * and returns the feedforward output with constant gravity compensation.
 *
 * @param U the unit type for the motion state
 * @param kG gravity gain - constant output to counteract gravity
 * @param kS static friction gain - minimum output to overcome static friction
 * @param kV velocity gain - output needed to maintain velocity
 * @param kA acceleration gain - output needed to accelerate
 * @return a [Function] that computes feedforward output from a motion state
 */
@JvmOverloads
fun <U : Unit<U>> elevatorFeedforward(
    kG: Double = 0.0,
    kS: Double = 0.0,
    kV: Double = 0.0,
    kA: Double = 0.0,
): Function<MotionState<U>, Double> = elevatorFeedforward(
    GravityFeedforwardParameters(kG, kS, kV, kA),
)

/**
 * Creates a [Function] that wraps an [ArmFeedforward] controller.
 *
 * The returned function takes a [MotionState] representing the desired state
 * and returns the feedforward output with angle-dependent gravity compensation
 * (using cosine of the position).
 *
 * @param U the unit type for the motion state
 * @param coefficients the feedforward parameters (kG, kS, kV, kA)
 * @return a [Function] that computes feedforward output from a motion state
 */
fun <U : Unit<U>> armFeedforward(coefficients: GravityFeedforwardParameters): Function<MotionState<U>, Double> {
    val ff = ArmFeedforward<U>(coefficients)
    return Function { ff.calculate(it) }
}

/**
 * Creates a [Function] that wraps an [ArmFeedforward] controller.
 *
 * The returned function takes a [MotionState] representing the desired state
 * and returns the feedforward output with angle-dependent gravity compensation
 * (using cosine of the position).
 *
 * @param U the unit type for the motion state
 * @param kG gravity gain - output to counteract gravity (multiplied by cos(position))
 * @param kS static friction gain - minimum output to overcome static friction
 * @param kV velocity gain - output needed to maintain angular velocity
 * @param kA acceleration gain - output needed to accelerate
 * @return a [Function] that computes feedforward output from a motion state
 */
@JvmOverloads
fun <U : Unit<U>> armFeedforward(
    kG: Double = 0.0,
    kS: Double = 0.0,
    kV: Double = 0.0,
    kA: Double = 0.0,
): Function<MotionState<U>, Double> = armFeedforward(
    GravityFeedforwardParameters(kG, kS, kV, kA),
)
