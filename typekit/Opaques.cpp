/* Generated from orogen/lib/orogen/templates/typekit/Opaques.cpp */

#include "Opaques.hpp"
#include <base/Opaques.hpp>

    /** Returns the intermediate value that is contained in \c real_type */
    /** Stores \c intermediate into \c real_type. \c intermediate is owned by \c
     * real_type afterwards. */
    /** Release ownership of \c real_type on the corresponding intermediate
     * pointer.
     */


void orogen_typekits::toIntermediate(::wrappers::KDLChain& intermediate, ::KDL::Chain const& real_type)
{
    intermediate = ::wrappers::KDLChain( real_type );
}

void orogen_typekits::fromIntermediate(::KDL::Chain& real_type, ::wrappers::KDLChain const& intermediate)
{
    std::vector< ::wrappers::KDLSegment>::const_iterator it = intermediate.segments.begin();
    for(; it != intermediate.segments.end(); ++it)
    {
        ::KDL::Segment segment;
        fromIntermediate(segment, *it);
        real_type.addSegment(segment);
    }
}


void orogen_typekits::toIntermediate(::wrappers::KDLFrame& intermediate, ::KDL::Frame const& real_type)
{
    intermediate = ::wrappers::KDLFrame( real_type );
}

void orogen_typekits::fromIntermediate(::KDL::Frame& real_type, ::wrappers::KDLFrame const& intermediate)
{
    fromIntermediate(real_type.p, intermediate.p);
    fromIntermediate(real_type.M, intermediate.M);
}


void orogen_typekits::toIntermediate(::wrappers::KDLFrame2& intermediate, ::KDL::Frame2 const& real_type)
{
    toIntermediate(intermediate.p, real_type.p);
    toIntermediate(intermediate.M, real_type.M);
}

void orogen_typekits::fromIntermediate(::KDL::Frame2& real_type, ::wrappers::KDLFrame2 const& intermediate)
{
    fromIntermediate(real_type.p, intermediate.p);
    fromIntermediate(real_type.M, intermediate.M);
}


void orogen_typekits::toIntermediate(::wrappers::KDLJacobian& intermediate, ::KDL::Jacobian const& real_type)
{
    toIntermediate(intermediate.data, real_type.data);
}

void orogen_typekits::fromIntermediate(::KDL::Jacobian& real_type, ::wrappers::KDLJacobian const& intermediate)
{
    fromIntermediate(real_type.data, intermediate.data);
}


void orogen_typekits::toIntermediate(::wrappers::KDLJntArray& intermediate, ::KDL::JntArray const& real_type)
{
    toIntermediate(intermediate.data, real_type.data);
}

void orogen_typekits::fromIntermediate(::KDL::JntArray& real_type, ::wrappers::KDLJntArray const& intermediate)
{
    fromIntermediate(real_type.data, intermediate.data);
}


void orogen_typekits::toIntermediate(::wrappers::KDLJoint& intermediate, ::KDL::Joint const& real_type)
{
    intermediate = ::wrappers::KDLJoint( real_type );
}

void orogen_typekits::fromIntermediate(::KDL::Joint& real_type, ::wrappers::KDLJoint const& intermediate)
{
    ::KDL::Vector origin, axis;
    orogen_typekits::fromIntermediate(origin, intermediate.origin);
    orogen_typekits::fromIntermediate(axis, intermediate.axis);

    real_type = KDL::Joint(intermediate.name, origin, axis, static_cast<KDL::Joint::JointType>(intermediate.type) );
}


void orogen_typekits::toIntermediate(::wrappers::KDLRigidBodyInertia& intermediate, ::KDL::RigidBodyInertia const& real_type)
{
    intermediate.m = real_type.getMass();
    toIntermediate(intermediate.I, real_type.getRotationalInertia());
    toIntermediate(intermediate.cog, real_type.getCOG());
}

void orogen_typekits::fromIntermediate(::KDL::RigidBodyInertia& real_type, ::wrappers::KDLRigidBodyInertia const& intermediate)
{
    ::KDL::Vector cog;
    fromIntermediate(cog, intermediate.cog);

    ::KDL::RotationalInertia inertia;
    fromIntermediate(inertia, intermediate.I);

    real_type = ::KDL::RigidBodyInertia(intermediate.m, cog, inertia);
}


void orogen_typekits::toIntermediate(::wrappers::KDLRotation& intermediate, ::KDL::Rotation const& real_type)
{
    intermediate = ::wrappers::KDLRotation( real_type );
}

void orogen_typekits::fromIntermediate(::KDL::Rotation& real_type, ::wrappers::KDLRotation const& intermediate)
{
    memcpy(real_type.data, intermediate.data, 9*sizeof(double));
}


void orogen_typekits::toIntermediate(::wrappers::KDLRotation2& intermediate, ::KDL::Rotation2 const& real_type)
{
    intermediate = ::wrappers::KDLRotation2( real_type );
}

void orogen_typekits::fromIntermediate(::KDL::Rotation2& real_type, ::wrappers::KDLRotation2 const& intermediate)
{
    real_type = ::KDL::Rotation2(intermediate.rotationInRad);
}


void orogen_typekits::toIntermediate(::wrappers::KDLRotationalInertia& intermediate, ::KDL::RotationalInertia const& real_type)
{
    memcpy(intermediate.data, real_type.data, 9*sizeof(double) );
}

void orogen_typekits::fromIntermediate(::KDL::RotationalInertia& real_type, ::wrappers::KDLRotationalInertia const& intermediate)
{
    memcpy(real_type.data, intermediate.data, 9*sizeof(double) );
}


void orogen_typekits::toIntermediate(::wrappers::KDLSegment& intermediate, ::KDL::Segment const& real_type)
{
    intermediate = ::wrappers::KDLSegment( real_type );
}

void orogen_typekits::fromIntermediate(::KDL::Segment& real_type, ::wrappers::KDLSegment const& intermediate)
{
    KDL::Joint joint;
    orogen_typekits::fromIntermediate(joint, intermediate.joint);

    KDL::Frame f_tip;
    orogen_typekits::fromIntermediate(f_tip, intermediate.f_tip);

    KDL::RigidBodyInertia inertia;
    orogen_typekits::fromIntermediate(inertia, intermediate.I);

    real_type = KDL::Segment( intermediate.name, joint, f_tip, inertia); 
}


void orogen_typekits::toIntermediate(::wrappers::KDLTwist& intermediate, ::KDL::Twist const& real_type)
{
    intermediate = ::wrappers::KDLTwist( real_type );
}

void orogen_typekits::fromIntermediate(::KDL::Twist& real_type, ::wrappers::KDLTwist const& intermediate)
{
    orogen_typekits::fromIntermediate(real_type.vel, intermediate.vel);
    orogen_typekits::fromIntermediate(real_type.rot, intermediate.rot);
}


void orogen_typekits::toIntermediate(::wrappers::KDLVector& intermediate, ::KDL::Vector const& real_type)
{
    intermediate = ::wrappers::KDLVector( real_type );
}

void orogen_typekits::fromIntermediate(::KDL::Vector& real_type, ::wrappers::KDLVector const& intermediate)
{
    memcpy(real_type.data, intermediate.data, 3*sizeof(double));
}


void orogen_typekits::toIntermediate(::wrappers::KDLVector2& intermediate, ::KDL::Vector2 const& real_type)
{
    intermediate = ::wrappers::KDLVector2( real_type );
}

void orogen_typekits::fromIntermediate(::KDL::Vector2& real_type, ::wrappers::KDLVector2 const& intermediate)
{
    real_type = KDL::Vector2(intermediate.data[0], intermediate.data[1]);
}

void orogen_typekits::toIntermediate(::wrappers::KDLWrench& intermediate, ::KDL::Wrench const& real_type)
{
    intermediate = ::wrappers::KDLWrench( real_type );
}

void orogen_typekits::fromIntermediate(::KDL::Wrench& real_type, ::wrappers::KDLWrench const& intermediate)
{
    orogen_typekits::fromIntermediate(real_type.force, intermediate.force);
    orogen_typekits::fromIntermediate(real_type.torque, intermediate.torque);
}



