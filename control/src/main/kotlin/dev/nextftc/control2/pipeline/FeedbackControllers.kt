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

import dev.nextftc.control2.feedback.PIDCoefficients
import dev.nextftc.control2.feedback.PIDController
import dev.nextftc.control2.model.AngularMotionState
import dev.nextftc.control2.model.LinearMotionState
import dev.nextftc.control2.model.MotionState
import dev.nextftc.units.Unit
import dev.nextftc.units.unittypes.Inches
import dev.nextftc.units.unittypes.InchesPerSecond
import dev.nextftc.units.unittypes.Radians
import dev.nextftc.units.unittypes.RadiansPerSecond
import kotlin.time.TimeSource

/**
 * Creates a [Function] that computes the error between a reference and measured [MotionState].
 *
 * The returned function takes a [Pair] of motion states (reference, measured) and returns
 * a new motion state representing the error (reference - measured) for each component
 * (position, velocity, acceleration).
 *
 * This is typically used as the first stage in a feedback control pipeline to compute
 * the error that will be fed into a PID controller.
 *
 * @param U the unit type for the motion states
 * @return a [Function] that computes the error motion state from (reference, measured) pairs
 */
fun <U : Unit<U>> error(): Function<Pair<MotionState<U>, MotionState<U>>, MotionState<U>> =
    Function { (reference, measured) ->
        reference - measured
    }

/**
 * Creates a [Function] that wraps a [PIDController] for positional control.
 *
 * The returned function takes a [LinearMotionState] representing the error
 * (difference between reference and measured states) and returns the PID control output.
 *
 * The error state's position is used as the error term, and the velocity is used
 * as the error derivative for the D term calculation.
 *
 * @param coefficients the PID coefficients to use
 * @param resetIntegralOnZeroCrossover whether to reset the integral when error crosses zero
 * @param timeSource the time source to use for calculating time deltas (defaults to system monotonic clock)
 * @return a [Function] that computes PID output from error state
 */
@JvmOverloads
fun positionPID(
    coefficients: PIDCoefficients,
    resetIntegralOnZeroCrossover: Boolean = true,
    timeSource: TimeSource.WithComparableMarks = TimeSource.Monotonic,
): Function<LinearMotionState, Double> {
    val controller = PIDController(coefficients, resetIntegralOnZeroCrossover)
    return Function {
        val timestamp = timeSource.markNow()
        val error = it.position.into(Inches)
        val errorDerivative = it.velocity.into(InchesPerSecond)
        controller.calculate(timestamp, error, errorDerivative)
    }
}

/**
 * Creates a [Function] that wraps a [PIDController] for positional control.
 *
 * The returned function takes a [LinearMotionState] representing the error
 * (difference between reference and measured states) and returns the PID control output.
 *
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 * @param resetIntegralOnZeroCrossover whether to reset the integral when error crosses zero
 * @param timeSource the time source to use for calculating time deltas
 * @return a [Function] that computes PID output from error state
 */
@JvmOverloads
fun positionPID(
    kP: Double,
    kI: Double = 0.0,
    kD: Double = 0.0,
    resetIntegralOnZeroCrossover: Boolean = true,
    timeSource: TimeSource.WithComparableMarks = TimeSource.Monotonic,
): Function<LinearMotionState, Double> = positionPID(
    PIDCoefficients(kP, kI, kD),
    resetIntegralOnZeroCrossover,
    timeSource,
)

/**
 * Creates a [Function] that wraps a [PIDController] for angular positional control.
 *
 * The returned function takes an [AngularMotionState] representing the error
 * (difference between reference and measured states) and returns the PID control output.
 *
 * The error state's position is used as the error term (in radians), and the velocity
 * is used as the error derivative for the D term calculation.
 *
 * @param coefficients the PID coefficients to use
 * @param resetIntegralOnZeroCrossover whether to reset the integral when error crosses zero
 * @param timeSource the time source to use for calculating time deltas (defaults to system monotonic clock)
 * @return a [Function] that computes PID output from angular error state
 */
@JvmOverloads
fun angularPID(
    coefficients: PIDCoefficients,
    resetIntegralOnZeroCrossover: Boolean = true,
    timeSource: TimeSource.WithComparableMarks = TimeSource.Monotonic,
): Function<AngularMotionState, Double> {
    val controller = PIDController(coefficients, resetIntegralOnZeroCrossover)
    return Function {
        val timestamp = timeSource.markNow()
        val error = it.position.into(Radians)
        val errorDerivative = it.velocity.into(RadiansPerSecond)
        controller.calculate(timestamp, error, errorDerivative)
    }
}

/**
 * Creates a [Function] that wraps a [PIDController] for angular positional control.
 *
 * The returned function takes an [AngularMotionState] representing the error
 * (difference between reference and measured states) and returns the PID control output.
 *
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 * @param resetIntegralOnZeroCrossover whether to reset the integral when error crosses zero
 * @param timeSource the time source to use for calculating time deltas
 * @return a [Function] that computes PID output from angular error state
 */
@JvmOverloads
fun angularPID(
    kP: Double,
    kI: Double = 0.0,
    kD: Double = 0.0,
    resetIntegralOnZeroCrossover: Boolean = true,
    timeSource: TimeSource.WithComparableMarks = TimeSource.Monotonic,
): Function<AngularMotionState, Double> = angularPID(
    PIDCoefficients(kP, kI, kD),
    resetIntegralOnZeroCrossover,
    timeSource,
)

/**
 * Creates a [Function] that wraps a [PIDController] for velocity control.
 *
 * The returned function takes a [Double] representing the velocity error
 * (difference between reference and measured velocity) and returns the PID control output.
 *
 * The error derivative is computed automatically from the change in error over time.
 *
 * @param coefficients the PID coefficients to use
 * @param resetIntegralOnZeroCrossover whether to reset the integral when error crosses zero
 * @param timeSource the time source to use for calculating time deltas (defaults to system monotonic clock)
 * @return a [Function] that computes PID output from velocity error
 */
@JvmOverloads
fun velocityPID(
    coefficients: PIDCoefficients,
    resetIntegralOnZeroCrossover: Boolean = true,
    timeSource: TimeSource.WithComparableMarks = TimeSource.Monotonic,
): Function<Double, Double> {
    val controller = PIDController(coefficients, resetIntegralOnZeroCrossover)
    return Function {
        val timestamp = timeSource.markNow()
        controller.calculate(timestamp, it, null)
    }
}

/**
 * Creates a [Function] that wraps a [PIDController] for velocity control.
 *
 * The returned function takes a [Double] representing the velocity error
 * (difference between reference and measured velocity) and returns the PID control output.
 *
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 * @param resetIntegralOnZeroCrossover whether to reset the integral when error crosses zero
 * @param timeSource the time source to use for calculating time deltas
 * @return a [Function] that computes PID output from velocity error
 */
@JvmOverloads
fun velocityPID(
    kP: Double,
    kI: Double = 0.0,
    kD: Double = 0.0,
    resetIntegralOnZeroCrossover: Boolean = true,
    timeSource: TimeSource.WithComparableMarks = TimeSource.Monotonic,
): Function<Double, Double> = velocityPID(
    PIDCoefficients(kP, kI, kD),
    resetIntegralOnZeroCrossover,
    timeSource,
)
