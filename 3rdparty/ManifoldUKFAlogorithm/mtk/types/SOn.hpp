/*
 *  Copyright (c) 2008--2011, Universitaet Bremen
 *  All rights reserved.
 *
 *  Author: Christoph Hertzberg <chtz@informatik.uni-bremen.de>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file mtk/types/SOn.hpp
 * @brief Standard Orthogonal Groups i.e.\ rotatation groups.
 */
#ifndef SON_H_
#define SON_H_


#include "vect.hpp"
#include "../src/mtkmath.hpp"

#include <Eigen/Geometry>

namespace MTK {


/**
 * Two-dimensional orientations represented as scalar.
 * There is no guarantee that the representing scalar is within any interval,
 * but the result of boxminus will always have magnitude @f$\le\pi @f$.
 */
template<class _scalar = double>
struct SO2 : public Eigen::Rotation2D<_scalar> {
	enum {DOF = 1, DIM = 2};
	
	typedef _scalar scalar;
	typedef Eigen::Rotation2D<scalar> base;
	typedef vect<DIM, scalar> vect_type;
	
	//using base::operator=;
	
	//! Construct from angle
	SO2(const scalar& angle = 0) : base(angle) {	}
	
	//! Construct from Eigen::Rotation2D
	SO2(const base& src) : base(src) {}
	
	/**
	 * Construct from 2D vector.
	 * Resulting orientation will rotate the first unit vector to point to vec.
	 */
	SO2(const vect_type &vec) : base(atan2(vec[1], vec[0])) {};
	
	
	//! Calculate @c this->inverse() * @c r
	SO2 operator%(const base &r) const {
		return base::inverse() * r;
	}

	//! Calculate @c this->inverse() * @c r
	template<class Derived>
	vect_type operator%(const Eigen::MatrixBase<Derived> &vec) const {
		return base::inverse() * vec;
	}
	
	//! Calculate @c *this * @c r.inverse()
	SO2 operator/(const SO2 &r) const {
		return *this * r.inverse();
	}

/*	//! Gets the angle as scalar.
	operator scalar() const {
		return base::angle();
	}
*/
	
	//! @name Manifold requirements
	//{
	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale=1) {
		base::angle() += scale * vec[0];
	}
	void boxminus(MTK::vectview<scalar, DOF> res, const SO2<scalar>& other) const {
		res[0] = MTK::normalize(base::angle() - other.angle(), scalar(MTK::pi));
	}
	//}
	
	friend std::ostream& operator<<(std::ostream &os, const SO2<scalar>& ang) {
		return os << ang.angle();
	}

	friend std::istream& operator>>(std::istream &is, SO2<scalar>& ang){
		return is >> ang.angle();
	}

};


/**
 * Three-dimensional orientations represented as Quaternion.
 * It is assumed that the internal Quaternion always stays normalized,
 * should this not be the case, call inherited member function @c normalize().
 */
template<class _scalar = double>
struct SO3 : public Eigen::Quaternion<_scalar> {
	enum {DOF = 3, DIM = 3};
	typedef _scalar scalar;
	typedef Eigen::Quaternion<scalar> base;
	typedef vect<DIM, scalar> vect_type;
	
	SO3 operator=(const SO3 &r)
	{
		this->w() = r.w();
		this->x() = r.x();
		this->y() = r.y();
		this->z() = r.z();
		return *this;
	}
	
	
	//! Calculate @c this->inverse() * @c r
	SO3 operator%(const base &r) const {
		return base::conjugate() * r;
	}
	
	//! Calculate @c this->inverse() * @c r
	template<class Derived>
	vect_type operator%(const Eigen::MatrixBase<Derived> &vec) const {
		return base::conjugate() * vec;
	}
	
	//! Calculate @c this * @c r.conjugate()
	SO3 operator/(const base& r) const {
		return *this * r.conjugate();
	}
	
	/**
	 * Construct from real part and three imaginary parts.
	 * Quaternion is normalized after construction.
	 */
	SO3(const scalar& w, const scalar& x, const scalar& y, const scalar& z) : base(w, x, y, z) {
		base::normalize();
	}
	
	/**
	 * Construct from Eigen::Quaternion.
	 * @note Non-normalized input may result result in spurious behavior.
	 */
	SO3(const base& src = base::Identity()) : base(src) {}
	
	/**
	 * Construct from rotation matrix.
	 * @note Invalid rotation matrices may lead to spurious behavior.
	 */
	template<class Derived>
	SO3(const Eigen::MatrixBase<Derived>& matrix) : base(matrix) {}
	
	//! @name Manifold requirements
	//{
	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale=1) {
		SO3 delta = exp(vec, scale);
		*this = delta * *this;
	}
	void boxminus(MTK::vectview<scalar, DOF> res, const SO3<scalar>& other) const {
		res = SO3::log(*this * other.conjugate());
	}
	//}
	
	friend std::ostream& operator<<(std::ostream &os, const SO3<scalar>& q){
		return os << q.coeffs().transpose() << " ";
	}
	friend std::istream& operator>>(std::istream &is, SO3<scalar>& q){
		for(int i=0; i<4; ++i)
			is >> q.coeffs()[i];
		q.normalize();
		return is;
	}
	
	//! @name Helper functions
	//{
	/**
	 * Calculate the exponential map. In matrix terms this would correspond 
	 * to the Rodrigues formula.
	 */
	static SO3 exp(MTK::vectview<const scalar, 3> dvec, scalar scale = 1){
		SO3 res;
		res.w() = MTK::exp<scalar, 3>(res.vec(), dvec, scalar(scale/2));
		return res;
	}
	/**
	 * Calculate the inverse of @c exp.
	 * Only guarantees that <code>exp(log(x)) == x </code>
	 */
	static typename base::Vector3 log(const SO3 &orient){
		typename base::Vector3 res;
		MTK::log<scalar, 3>(res, orient.w(), orient.vec(), scalar(2), true);
		return res;
	}
	//}
};

}  // namespace MTK

#endif /*SON_H_*/

