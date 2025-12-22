/*
 * Copyright (c) 2025 NextFTC Team
 *
 *  Use of this source code is governed by an BSD-3-clause
 *  license that can be found in the LICENSE.md file at the root of this repository or at
 *  https://opensource.org/license/bsd-3-clause.
 */

package dev.nextftc.control2.model

import dev.nextftc.linalg.Vector
import dev.nextftc.linalg.makeVector
import dev.nextftc.units.Measure
import dev.nextftc.units.Unit
import dev.nextftc.units.measuretypes.Angle
import dev.nextftc.units.measuretypes.AngularAcceleration
import dev.nextftc.units.measuretypes.AngularVelocity
import dev.nextftc.units.measuretypes.Distance
import dev.nextftc.units.measuretypes.LinearAcceleration
import dev.nextftc.units.measuretypes.LinearVelocity
import dev.nextftc.units.measuretypes.Per
import dev.nextftc.units.unittypes.AngleUnit
import dev.nextftc.units.unittypes.DistanceUnit
import dev.nextftc.units.unittypes.PerUnit
import dev.nextftc.units.unittypes.Radians
import dev.nextftc.units.unittypes.RadiansPerSecond
import dev.nextftc.units.unittypes.RadiansPerSecondSquared
import dev.nextftc.units.unittypes.TimeUnit
import dev.nextftc.units.unittypes.inches
import dev.nextftc.units.unittypes.inchesPerSecond
import dev.nextftc.units.unittypes.inchesPerSecondSquared

/**
 * The kinematic state of a system, parameterized by position unit type.
 *
 * Represents the complete motion state of a system at a point in time, including
 * position, velocity, and acceleration. Velocity is derived as position per time,
 * and acceleration as velocity per time.
 *
 * This interface is generic over the position unit type, allowing it to be used
 * for both linear motion (with [DistanceUnit]) and angular motion (with [AngleUnit]).
 *
 * @param U The position unit type (e.g., [DistanceUnit] for linear motion, [AngleUnit] for angular motion)
 * @see LinearMotionState
 * @see AngularMotionState
 */
interface MotionState<U : Unit<U>> {
    /** The position of the system. */
    val position: Measure<U>

    /** The velocity of the system (position per time). */
    val velocity: Per<U, TimeUnit>

    /** The acceleration of the system (velocity per time). */
    val acceleration: Per<PerUnit<U, TimeUnit>, TimeUnit>

    /**
     * Creates a copy of this motion state with optionally modified values.
     *
     * @param position The new position (defaults to current position)
     * @param velocity The new velocity (defaults to current velocity)
     * @param acceleration The new acceleration (defaults to current acceleration)
     * @return A new [MotionState] with the specified values
     */
    fun copy(
        position: Measure<U> = this.position,
        velocity: Per<U, TimeUnit> = this.velocity,
        acceleration: Per<PerUnit<U, TimeUnit>, TimeUnit> = this.acceleration,
    ): MotionState<U>

    /**
     * Creates a copy of this motion state with optionally modified values using raw doubles.
     *
     * The double values are interpreted in the same units as the current state's
     * position, velocity, and acceleration.
     *
     * @param position The new position magnitude (defaults to current position magnitude)
     * @param velocity The new velocity magnitude (defaults to current velocity magnitude)
     * @param acceleration The new acceleration magnitude (defaults to current acceleration magnitude)
     * @return A new [MotionState] with the specified values
     */
    fun copy(
        position: Double = this.position.magnitude,
        velocity: Double = this.velocity.magnitude,
        acceleration: Double = this.acceleration.magnitude,
    ) = copy(
        this.position.unit.of(position),
        this.velocity.unit.of(velocity),
        this.acceleration.unit.of(acceleration),
    )

    /**
     * Converts this motion state to a 3-element vector.
     *
     * The vector contains [position, velocity, acceleration] in that order,
     * using the magnitude values in the current units.
     *
     * @return A [Vector] containing the position, velocity, and acceleration magnitudes
     */
    fun toVector() = makeVector(position.magnitude, velocity.magnitude, acceleration.magnitude)

    /**
     * Returns the negation of this motion state.
     *
     * All components (position, velocity, acceleration) are negated.
     *
     * @return A new [MotionState] with all components negated
     */
    operator fun unaryMinus() = copy(position = -position, velocity = -velocity, acceleration = -acceleration)

    /**
     * Adds another motion state to this one, component-wise.
     *
     * @param other The motion state to add
     * @return A new [MotionState] with each component being the sum of the corresponding components
     */
    operator fun plus(other: MotionState<U>) = copy(
        position = position + other.position,
        velocity = velocity + other.velocity,
        acceleration = acceleration + other.acceleration,
    )

    /**
     * Subtracts another motion state from this one, component-wise.
     *
     * This is useful for computing error states (reference - measured).
     *
     * @param other The motion state to subtract
     * @return A new [MotionState] with each component being the difference of the corresponding components
     */
    operator fun minus(other: MotionState<U>) = copy(
        position = position - other.position,
        velocity = velocity - other.velocity,
        acceleration = acceleration - other.acceleration,
    )

    /**
     * Multiplies this motion state by a scalar, component-wise.
     *
     * @param scalar The scalar to multiply by
     * @return A new [MotionState] with each component multiplied by the scalar
     */
    operator fun times(scalar: Double) = copy(
        position = position * scalar,
        velocity = velocity * scalar,
        acceleration = acceleration * scalar,
    )

    /**
     * Multiplies this motion state by a scalar, component-wise.
     *
     * @param scalar The scalar to multiply by (converted to Double)
     * @return A new [MotionState] with each component multiplied by the scalar
     */
    operator fun times(scalar: Number) = times(scalar.toDouble())

    /**
     * Divides this motion state by a scalar, component-wise.
     *
     * @param divisor The scalar to divide by
     * @return A new [MotionState] with each component divided by the divisor
     */
    operator fun div(divisor: Double) = copy(
        position = position / divisor,
        velocity = velocity / divisor,
        acceleration = acceleration / divisor,
    )

    /**
     * Divides this motion state by a scalar, component-wise.
     *
     * @param divisor The scalar to divide by (converted to Double)
     * @return A new [MotionState] with each component divided by the divisor
     */
    operator fun div(divisor: Number) = div(divisor.toDouble())
}

/**
 * The linear motion state of an object (position in distance units).
 *
 * @property position The position of the object.
 * @property velocity The velocity of the object.
 * @property acceleration The acceleration of the object.
 */
data class LinearMotionState @JvmOverloads constructor(
    override val position: Distance = 0.0.inches,
    override val velocity: LinearVelocity = 0.0.inchesPerSecond,
    override val acceleration: LinearAcceleration = 0.0.inchesPerSecondSquared,
) : MotionState<DistanceUnit> {

    /**
     * Creates a LinearMotionState with the given position, velocity, and acceleration.
     *
     * @param position Position, in inches
     * @param velocity Velocity, in inches per second
     * @param acceleration Acceleration, in inches per second squared
     */
    constructor(
        position: Double = 0.0,
        velocity: Double = 0.0,
        acceleration: Double = 0.0,
    ) : this(
        position.inches,
        velocity.inchesPerSecond,
        acceleration.inchesPerSecondSquared,
    )

    override fun copy(
        position: Measure<DistanceUnit>,
        velocity: Per<DistanceUnit, TimeUnit>,
        acceleration: Per<PerUnit<DistanceUnit, TimeUnit>, TimeUnit>,
    ): LinearMotionState = LinearMotionState(
        this.position.unit.of(position.into(this.position.unit)),
        this.velocity.unit.of(velocity.into(this.velocity.unit)) as LinearVelocity,
        this.acceleration.unit.of(acceleration.into(this.acceleration.unit)) as LinearAcceleration,
    )

    companion object {
        val ZERO = LinearMotionState(0.0, 0.0, 0.0)
    }
}

/**
 * The angular motion state of an object (position in angle units).
 *
 * @property position The angular position of the object.
 * @property velocity The angular velocity of the object.
 * @property acceleration The angular acceleration of the object.
 */
data class AngularMotionState @JvmOverloads constructor(
    override val position: Angle = Radians.of(0.0),
    override val velocity: AngularVelocity = RadiansPerSecond.of(0.0),
    override val acceleration: AngularAcceleration = RadiansPerSecondSquared.of(0.0),
) : MotionState<AngleUnit> {

    /**
     * Creates an AngularMotionState with the given position, velocity, and acceleration.
     *
     * @param position Position, in radians
     * @param velocity Velocity, in radians per second
     * @param acceleration Acceleration, in radians per second squared
     */
    constructor(
        position: Double = 0.0,
        velocity: Double = 0.0,
        acceleration: Double = 0.0,
    ) : this(
        Radians.of(position),
        RadiansPerSecond.of(velocity),
        RadiansPerSecondSquared.of(acceleration),
    )

    override fun copy(
        position: Measure<AngleUnit>,
        velocity: Per<AngleUnit, TimeUnit>,
        acceleration: Per<PerUnit<AngleUnit, TimeUnit>, TimeUnit>,
    ): AngularMotionState = AngularMotionState(
        this.position.unit.of(position.into(this.position.unit)),
        this.velocity.unit.of(velocity.into(this.velocity.unit)) as AngularVelocity,
        this.acceleration.unit.of(acceleration.into(this.acceleration.unit)) as AngularAcceleration,
    )

    companion object {
        val ZERO = AngularMotionState(0.0, 0.0, 0.0)
    }
}
