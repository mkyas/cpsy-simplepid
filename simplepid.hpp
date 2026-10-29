
#pragma once

#ifndef CPSY_SIMPLE_PID
#define CPSY_SIMPLE_PID

#include <algorithm>
#include <chrono>
#include <limits>



/*
 * A simple PID controller.
 *
 * typename T is the type of the setpoint and the scalars.
 */
template<typename T>
class simplepid
{
	public:
		simplepid() = delete;
		~simplepid() = default;
		simplepid(const T& _Kp, const T& _Ki, const T& _Kd);
		simplepid(const simplepid&) = delete;
		simplepid& operator=(const simplepid&) = delete;
		simplepid(simplepid&&) = delete;
		simplepid& operator=(const simplepid&&) = delete;

		void set_parameters(const T& _Kp, const T& _Ki, const T& _Kd) noexcept;
		void set_output_range(const T&, const T&) noexcept;
		void set_setpoint(const T&) noexcept;
		void reset(void) noexcept;

		inline T operator()(const T& actual, const std::chrono::time_point<std::chrono::steady_clock> &now) noexcept;
		inline T operator()(const T& input) noexcept;

	private:
		T Kp;  // The value of the propotional gain \(K_P\).
		T Ki;  // The value of the integral gain \(K_I\).
		T Kd;  // The value of the derivative gain \(K_D\).
		T setpoint;    // The setpoint the PID tries to achieve
		T min_output;  // lower output bound
		T max_output;  // upper output bound

		T last_input;  // last imput value
		T integral;    // current integral value
		std::chrono::time_point<std::chrono::steady_clock> last_time; // last update time.
};


/*
 * Constructor.
 */
template <typename T>
simplepid<T>::simplepid(const T& _Kp, const T& _Ki, const T& _Kd) :
	Kp{_Kp}, Ki{_Ki}, Kd{_Kd}, setpoint{},
	min_output{std::numeric_limits<T>::min()}, max_output{std::numeric_limits<T>::max()},
	last_input{}, integral{}, last_time{std::chrono::steady_clock::now()}
{
	// Do nothing.
}


/*
 * Change the parameters.
 */
template <typename T>
void simplepid<T>::set_parameters(const T& _Kp, const T& _Ki, const T& _Kd) noexcept
{
	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
	reset();
}


/*
 * Set the output range.
 */
template <typename T>
void simplepid<T>::set_output_range(const T& _min, const T& _max) noexcept
{
	min_output = _min;
	max_output = _max;
}


/*
 * Change the setpoint.
 */
template <typename T>
void simplepid<T>::set_setpoint(const T& _setpoint) noexcept
{
	setpoint = _setpoint;
}


/*
 * Reset the controller state.
 */
template <typename T>
void simplepid<T>::reset(void) noexcept
{
	integral = 0;
}


/*
 * update the controller state.
 */
template<typename T>
inline T simplepid<T>::operator()(const T& input, const std::chrono::time_point<std::chrono::steady_clock> &now) noexcept
{
	const std::chrono::duration<T> &dt = now - last_time; // Duration is in seconds.
	last_time = now;

	const T error = setpoint - input;
	const T new_integral = integral + Ki * error * dt.count();
	const T d_error = (last_input - input) / dt.count();

	const T output = Kp * error + new_integral + Kd * d_error;

	if (__builtin_expect(output <= min_output, false))
		return min_output;
	if (__builtin_expect(output >= max_output, false))
		return max_output;
	integral = std::clamp(new_integral, min_output, max_output);
	return output;
}


/*
 * update the controller state.
 */

template<typename T>
inline T simplepid<T>::operator()(const T& input) noexcept
{
	const auto now = std::chrono::steady_clock::now();
	return operator()(input, now);
}

#endif
