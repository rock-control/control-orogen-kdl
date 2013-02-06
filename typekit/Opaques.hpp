/* Generated from orogen/lib/orogen/templates/typekit/Opaques.hpp */

#ifndef __OROGEN_GENERATED_kdl_USER_MARSHALLING_HH
#define __OROGEN_GENERATED_kdl_USER_MARSHALLING_HH

#include <kdl/Types.hpp>

namespace orogen_typekits
{
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::KDLChain& intermediate, ::KDL::Chain const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::KDL::Chain& real_type, ::wrappers::KDLChain const& intermediate);
        
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::KDLFrame& intermediate, ::KDL::Frame const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::KDL::Frame& real_type, ::wrappers::KDLFrame const& intermediate);
        
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::KDLFrame2& intermediate, ::KDL::Frame2 const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::KDL::Frame2& real_type, ::wrappers::KDLFrame2 const& intermediate);
        
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::KDLJntArray& intermediate, ::KDL::JntArray const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::KDL::JntArray& real_type, ::wrappers::KDLJntArray const& intermediate);
        
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::KDLJoint& intermediate, ::KDL::Joint const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::KDL::Joint& real_type, ::wrappers::KDLJoint const& intermediate);
        
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::KDLRigidBodyInertia& intermediate, ::KDL::RigidBodyInertia const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::KDL::RigidBodyInertia& real_type, ::wrappers::KDLRigidBodyInertia const& intermediate);
        
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::KDLRotation& intermediate, ::KDL::Rotation const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::KDL::Rotation& real_type, ::wrappers::KDLRotation const& intermediate);
        
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::KDLRotation2& intermediate, ::KDL::Rotation2 const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::KDL::Rotation2& real_type, ::wrappers::KDLRotation2 const& intermediate);
        
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::KDLRotationalInertia& intermediate, ::KDL::RotationalInertia const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::KDL::RotationalInertia& real_type, ::wrappers::KDLRotationalInertia const& intermediate);
        
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::KDLSegment& intermediate, ::KDL::Segment const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::KDL::Segment& real_type, ::wrappers::KDLSegment const& intermediate);
        
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::KDLTwist& intermediate, ::KDL::Twist const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::KDL::Twist& real_type, ::wrappers::KDLTwist const& intermediate);
        
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::KDLVector& intermediate, ::KDL::Vector const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::KDL::Vector& real_type, ::wrappers::KDLVector const& intermediate);
        
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::KDLVector2& intermediate, ::KDL::Vector2 const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::KDL::Vector2& real_type, ::wrappers::KDLVector2 const& intermediate);
        
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::KDLWrench& intermediate, ::KDL::Wrench const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::KDL::Wrench& real_type, ::wrappers::KDLWrench const& intermediate);
        
    // ADD custom functions for the eigen convertion
    template<typename T, int EIGEN_OPTIONS, int EIGEN_MAX_ROWS, int EIGEN_MAX_COLS>
    void toIntermediate(std::vector<T>& intermediate, Eigen::Matrix<T, Eigen::Dynamic, 1, EIGEN_OPTIONS, EIGEN_MAX_ROWS, EIGEN_MAX_COLS> const& real)
    {
        typedef Eigen::Matrix<T,Eigen::Dynamic,1,EIGEN_OPTIONS,EIGEN_MAX_ROWS,EIGEN_MAX_COLS> EigenVector;

        intermediate.data.resize(real.size(),0.0);

        Eigen::Map<EigenVector> m(&(intermediate.data[0]),real.size());
        m = real;
    }

    template<typename T, int EIGEN_OPTIONS, int EIGEN_MAX_ROWS, int EIGEN_MAX_COLS>
    void fromIntermediate(Eigen::Matrix<T, Eigen::Dynamic, 1, EIGEN_OPTIONS, EIGEN_MAX_ROWS, EIGEN_MAX_COLS>& real, std::vector<T> const& intermediate)
    {
        typedef const Eigen::Matrix<T,Eigen::Dynamic,1,EIGEN_OPTIONS,EIGEN_MAX_ROWS,EIGEN_MAX_COLS> EigenVector;

        Eigen::Map<EigenVector> m(&(intermediate.data[0]),intermediate.data.size());
        real = m;
    }
}

#endif

