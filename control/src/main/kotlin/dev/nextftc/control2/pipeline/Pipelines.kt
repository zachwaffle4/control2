/*
 * Copyright (c) 2025 NextFTC Team
 *
 *  Use of this source code is governed by an BSD-3-clause
 *  license that can be found in the LICENSE.md file at the root of this repository or at
 *  https://opensource.org/license/bsd-3-clause.
 */

package dev.nextftc.control2.pipeline

/**
 * A functional interface representing a transformation from type [T] to type [U].
 *
 * Functions can be composed together using [Composer]s via the [then] method,
 * enabling the creation of complex data processing pipelines.
 *
 * @param T the input type
 * @param U the output type
 */
fun interface Function<T, U> {
    /**
     * Applies this function to the given input.
     *
     * @param input the input value
     * @return the transformed output value
     */
    fun apply(input: T): U

    /**
     * Composes this function with the given [Composer] to create a new function.
     *
     * @param V the new input type after composition
     * @param W the new output type after composition
     * @param composer the composer that transforms this function
     * @return a new function resulting from the composition
     */
    fun <V, W> then(composer: Composer<T, U, V, W>) = composer.apply(this)
}

private val IDENTITY = Function<Any?, Any?> { x -> x }

/**
 * Returns an identity function that returns its input unchanged.
 *
 * @param T the type of input and output
 * @return an identity function
 */
@Suppress("UNCHECKED_CAST")
fun <T> identity() = IDENTITY as Function<T, T>

/**
 * A functional interface that transforms a [Function] into another [Function].
 *
 * Composers are used to modify or wrap existing functions, enabling
 * transformations like mapping outputs, changing input sources, or
 * adding side effects.
 *
 * @param T the input type of the original function
 * @param U the output type of the original function
 * @param V the input type of the resulting function
 * @param W the output type of the resulting function
 */
fun interface Composer<T, U, V, W> {
    /**
     * Applies this composer to transform the given function.
     *
     * @param function the function to transform
     * @return the transformed function
     */
    fun apply(function: Function<T, U>): Function<V, W>
}

/**
 * Creates a composer that maps the output of a function using the given [mapper].
 *
 * This is useful for transforming the result of a function without changing its input type.
 *
 * @param T the input type (preserved)
 * @param U the original output type
 * @param V the new output type after mapping
 * @param mapper the function to apply to the output
 * @return a composer that maps function outputs
 */
fun <T, U, V> map(mapper: Function<U, V>): Composer<T, U, T, V> =
    Composer { function -> Function { x -> mapper.apply(function.apply(x)) } }

/**
 * Creates a composer that uses a different [source] function to provide input.
 *
 * This allows chaining functions where the output of [source] becomes
 * the input to the composed function.
 *
 * @param T the input type of the source function
 * @param U the output type of the source function (and input type of the target function)
 * @param V the output type of the target function
 * @param source the function to use as the input source
 * @return a composer that chains the source with the target function
 */
fun <T, U, V> use(source: Function<T, U>) = Composer<U, V, T, V> { function ->
    source.then(map(function))
}

/**
 * Creates a function from a supplier that takes no meaningful input.
 *
 * This is useful for creating pipeline sources that generate values
 * from external sources like sensors or state.
 *
 * @param T the type of value produced
 * @param supplier the supplier function that produces values
 * @return a function that wraps the supplier
 */
fun <T> source(supplier: () -> T) = Function<Unit, T> { supplier() }

/**
 * Convenience extension to invoke a source function (a function with [Unit] input).
 *
 * @param T the output type
 * @return the result of invoking this source function
 */
fun <T> Function<Unit, T>.get() = apply(Unit)
