/**
 * @file TargetTracker.hpp
 * Linear Discrete Kalman filter
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once

#include <matrix/math.hpp>
#include <stdint.h>

// M = number of states
// N = number of measurements
template<uint8_t M, uint8_t N>
class KalmanFilter
{
friend class KalmanTest;
public:
	KalmanFilter() {};

	/**
	 * Initialize Kalman filter
	 *
	 * @param f   dynamic matrix F
	 * @param w   standard deviation of system noise
	 * @param h   design (measurement) matrix H
	 * @param v   standard deviation of measurement noise v
	 * @param x0  initial state x(k=0)
	 * @param p0  standard deviation of initial state
	 * @param dt  update period Delta t
	 */
	void init(const matrix::SquareMatrix<float, M> &f,
		  const matrix::Vector<float, M> &w,
		  const matrix::Matrix<float, N, M> &h,
		  const matrix::Vector<float, M> &x0,
		  const matrix::Vector<float, M> &p0,
		  float dt)
	{
		// --------------------------------------------------------------------------
		// TODO initialize member functions _h, _dt, _f and _x
		// --------------------------------------------------------------------------
		_h = h;
		_dt = dt;
		_f = f;
		_x = x0;

		// --------------------------------------------------------------------------
		// TODO set covariance matrix of estimation P(k=0)
		// --------------------------------------------------------------------------
		_p.setZero();
		for(int i = 0; i < M; i++) {
			_p(i, i) = pow(p0(i), 2);
		}

		// Set system noise
		set_system_noise(w);
	};

	/**
	 * Set system noise
	 * This calculates the state transition matrix _phi
	 * and the covariance matrix of the system noise _q
	 *
	 * @param w   standard deviation of system noise w
	 */
	void set_system_noise(const matrix::Vector<float, M> &w)
	{
		// Compute auxillary matrix A
		matrix::SquareMatrix<float, 2 * M> A = compute_matrix_A(w, _f, _dt);

		// Compute auxillary matrix B
		matrix::SquareMatrix<float, 2 * M> B = compute_matrix_B(A);

		// Compute phi
		_phi = compute_phi(B);

		// Compute Q
		_q = compute_q(B, _phi);
	}

	/** Perform a prediction step of the filter */
	void predict()
	{
		// update _x
		_x = predict_x(_phi, _x);
		// update _p
		_p = predict_p(_phi, _q, _p);
	};

	/**
	 * Perform a correction (update) step of the filter
	 *
	 * @param z    measurement
	 * @param v    covariance matrix of measurement
	 */
	void correct(const matrix::Vector<float, N> &z, const matrix::SquareMatrix<float, N> &r)
	{
		// Compute Kalman gains
		matrix::Matrix<float, M, N> k = compute_kalman_gain(_h, _p, r);

		// Update state estimation
		_x = compute_state_estimation(_x, z, k, _h);

		// Update state covariance estimation
		_p = compute_state_covariance_estimation(_p, k, _h);
	}

	/**
	 * Compute the new predicted state x.
	 * @param phi state transition matrix
	 * @param x previous state
	 * @return computed new predicted state
	 */
	static matrix::Vector<float, M> predict_x(
		matrix::SquareMatrix<float, M> &phi,
		matrix::Vector<float, M> &x)
	{
		// ------------------------------------------
		// TODO perform prediction: update x
		// ------------------------------------------

		return phi * x;
	}


	/**
	 * Compute the new predicted covariance matrix.
	 * @param phi state transition matrix
	 * @param q covariance matrix of the system noise
	 * @param p previous covariance matrix
	 * @return computed new predicted covariance matrix
	 */
	static matrix::SquareMatrix<float, M> predict_p(
		matrix::SquareMatrix<float, M> &phi,
		matrix::SquareMatrix<float, M> &q,
		matrix::SquareMatrix<float, M> &p)
	{
		// ------------------------------------------
		// TODO perform prediction: update _p
		// ------------------------------------------
		return phi * p * phi.T() + q;
	}

	/**
	 * Correct the kalman gain.
	 * @param h measurement (or design) matrix
	 * @param p covariance matrix
	 * @param r covariance matrix of the measurement
	 * @return computed kalman gain
	 */
	static matrix::Matrix<float, M, N> compute_kalman_gain(
		const matrix::Matrix<float, N, M> &h,
		const matrix::SquareMatrix<float, M> &p,
		const matrix::SquareMatrix<float, N> &r)
	{
		// -------------------------------------------------
		// TODO compute kalman gain k (weight/trust of measurement)
		// -------------------------------------------------
		matrix::SquareMatrix<float, N> temp = h * p * h.T() + r;
		return p * h.T() * matrix::inv(temp);
	}

	/**
	 * Correct state estimation.
	 * @param x previous state estimation
	 * @param z measurement
	 * @param k kalman gains matrix
	 * @param h measurement (or design) matrix
	 * @return corrected state estimation
	 */
	static matrix::Vector<float, M> compute_state_estimation(
		const matrix::Vector<float, M> &x,
		const matrix::Vector<float, N> &z,
		const matrix::Matrix<float, M, N> &k,
		const matrix::Matrix<float, N, M> &h)
	{
		// --------------------------------------------------
		// TODO return updated state estimation
		// --------------------------------------------------
		return x + k * (z - h * x);
	}

	/**
	 * Correct the state covariance estimation.
	 * @param p covariance matrix
	 * @param k kalman gains
	 * @param h measurement (or design) matrix
	 * @return corrected state covariance estimation
	 */
	static matrix::SquareMatrix<float, M> compute_state_covariance_estimation(
		const matrix::SquareMatrix<float, M> &p,
		const matrix::Matrix<float, M, N> &k,
		const matrix::Matrix<float, N, M> &h)
	{
		// --------------------------------------------------
		// TODO return updated estimation of state covariance
		// --------------------------------------------------
		matrix::Matrix<float, M, M> i;
		i.setIdentity();

		return (i - k * h) * p;
	}

	/**
	 * Compute auxillary matrix A.
	 * @param w standard deviation of system noise w
	 * @param f dynamic matrix
	 * @param dt time increment
	 * @return auxillary matrix A
	 */
	static matrix::SquareMatrix<float, 2 * M> compute_matrix_A(
		const matrix::Vector<float, M>& w,
		const matrix::SquareMatrix<float, M>& f,
		float dt)
	{
		// ----------------------------------------
		// TODO compute the auxillary matrix A
		// ----------------------------------------
		matrix::SquareMatrix<float, 2 * M> g;
		g.setZero();

		for(int i = 0; i < M; i++) {
			g(i, i) = w(i);
		}

		matrix::SquareMatrix<float, 2 * M> a;
		a.setZero();

		for(int i = 0; i < M; i++) {
			for(int j = 0; j < M; j++) {
				a(i,j) = (-f)(i, j);
				a(i + M,j + M) = (f.T())(i, j);
				a(i,j + M) = (g * g.T())(i, j);
			}
		}

		return a*dt;
	}

	/**
	 * Compute auxillary matrix B.
	 * @param A auxillary matrix A
	 * @return auxillary matrix B
	 */
	static matrix::SquareMatrix<float, 2 * M> compute_matrix_B(matrix::SquareMatrix<float, 2 * M> A)
	{
		// ------------------------------------------------
		// TODO compute the auxillary matrix B
		// ------------------------------------------------
		return matrix::expm(A);
	}

	/**
	 * Compute the state transition matrix phi.
	 * @param B auxillary matrix B
	 * @return computed state transition matrix phi
	 */
	static matrix::SquareMatrix<float, M> compute_phi(matrix::SquareMatrix<float, 2 * M> B)
	{
		// ------------------------------------------------
		// TODO compute phi
		// ------------------------------------------------
		matrix::SquareMatrix<float, M> phi;
		phi.setZero();

		for(int i = 0; i < M; i++) {
			for(int j = 0; j < M; j++) {
				phi(i, j) = B(i + M, j + M);
			}
		}

		return phi.T();
	}

	/**
	 * Compute covariance matrix of the system noise q.
	 * @param B auxillary matrix B
	 * @param phi the state transition matrix phi
	 * @return computed covariance matrix of the system noise
	 */
	static matrix::SquareMatrix<float, M> compute_q(
		matrix::SquareMatrix<float, 2 * M> B,
		matrix::SquareMatrix<float, M> phi)
	{
		// ------------------------------------------------
		// TODO compute phi
		// ------------------------------------------------
		matrix::SquareMatrix<float, M> B12;
		B12.setZero();

		for(int i = 0; i < M; i++) {
			for(int j = 0; j < M; j++) {
				B12(i, j) = B(i, j + M);
			}
		}
		return phi * B12;
	}


	/** @return estimated state */
	const matrix::Vector<float, M> &get_state_estimate() const
	{
		// ------------------------------------
		// TODO return state estimation
		// ------------------------------------
		return _x; // replace this line
	};

	/** @return variance (std^2) of state estimation */
	matrix::Vector<float, M> get_state_variances() const
	{
		// -----------------------------------------
		// TODO return variance of state estimation
		//      only diagonal elements
		// -----------------------------------------
		/*
		matrix::Vector<float, M> var;
		var.setZero();

		for(int i = 0; i < M; i++) {
			var(i) = _p(i, i);
		}
		*/

		return _p.diag(); // replace this line
	}

private:
	/** Covariance of system (model) noise [constant] */
	matrix::SquareMatrix<float, M> _q;
	/** measurement (design) matrix [constant] */
	matrix::Matrix<float, N, M> _h;
	/** state transition matrix [constant] */
	matrix::SquareMatrix<float, M> _phi;

	/** Dynamic matrix (only stored to recalculated phi and q in setSystemNoise(..)) */
	matrix::SquareMatrix<float, M> _f;
	/** time increment (only stored to recalculated phi and q in setSystemNoise(..)) */
	float _dt;

	/** A posteriori (estimated) state covariance */
	matrix::SquareMatrix<float, M> _p;
	/** A posteriori (estimated) state */
	matrix::Vector<float, M> _x;
};
