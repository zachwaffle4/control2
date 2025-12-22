/*
 * Copyright (c) 2025 NextFTC Team
 *
 *  Use of this source code is governed by an BSD-3-clause
 *  license that can be found in the LICENSE.md file at the root of this repository or at
 *  https://opensource.org/license/bsd-3-clause.
 */

@file:JvmName("Profiles")

package dev.nextftc.control2.pipeline

import dev.nextftc.control2.model.MotionState
import dev.nextftc.control2.profiles.TrapezoidProfile
import dev.nextftc.control2.profiles.TrapezoidProfileConstraints
import dev.nextftc.units.Unit
import kotlin.time.TimeSource

/**
 * Creates a [Function] that generates a trapezoidal motion profile.
 *
 * The returned function takes no meaningful input ([kotlin.Unit]) and returns a [MotionState]
 * representing the current target state along the profile. Each call advances the profile
 * based on the elapsed time since the first call.
 *
 * The profile starts from a zero state (position, velocity, and acceleration all zero)
 * and moves toward the goal state while respecting the velocity and acceleration constraints.
 *
 * @param U the unit type for the motion state
 * @param constraints the [TrapezoidProfileConstraints] defining maximum velocity and acceleration
 * @param goal a supplier function that returns the goal [MotionState] (allows dynamic goals)
 * @param timeSource the time source to use for calculating elapsed time (defaults to system monotonic clock)
 * @return a [Function] that returns the current profile state when invoked
 */
@JvmOverloads
fun <U: Unit<U>> trapezoidal(
    constraints: TrapezoidProfileConstraints<U>,
    goal: () -> MotionState<U>,
    timeSource: TimeSource.WithComparableMarks = TimeSource.Monotonic
) : Function<kotlin.Unit, MotionState<U>> {
    val profile = TrapezoidProfile(constraints)
    val zero = goal().copy(0.0, 0.0, 0.0)
    return Function { profile.calculate(timeSource.markNow(), zero, goal()) }
}

/**
 * Creates a [Function] that generates a trapezoidal motion profile.
 *
 * The returned function takes no meaningful input ([kotlin.Unit]) and returns a [MotionState]
 * representing the current target state along the profile. Each call advances the profile
 * based on the elapsed time since the first call.
 *
 * The profile starts from a zero state (position, velocity, and acceleration all zero)
 * and moves toward the goal state while respecting the velocity and acceleration constraints.
 *
 * @param U the unit type for the motion state
 * @param constraints the [TrapezoidProfileConstraints] defining maximum velocity and acceleration
 * @param goal the fixed goal [MotionState] to reach
 * @param timeSource the time source to use for calculating elapsed time (defaults to system monotonic clock)
 * @return a [Function] that returns the current profile state when invoked
 */
@JvmOverloads
fun <U: Unit<U>> trapezoidal(
    constraints: TrapezoidProfileConstraints<U>,
    goal: MotionState<U>,
    timeSource: TimeSource.WithComparableMarks = TimeSource.Monotonic
) = trapezoidal(constraints, { goal }, timeSource)