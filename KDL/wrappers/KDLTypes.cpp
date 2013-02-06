#include "KDLTypes.hpp"
#include <kdl/frames.hpp>
#include <kdl/joint.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/rotationalinertia.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/segment.hpp>
#include <kdl/chain.hpp>

#include <string.h>

namespace wrappers
{

KDLVector::KDLVector(::KDL::Vector const& v)
{
    memcpy(data, v.data, 3*sizeof(double) );
}

KDLVector2::KDLVector2(::KDL::Vector2 const& v)
{
    data[0] = v.x();
    data[1] = v.y();
}

KDLRotation::KDLRotation(::KDL::Rotation const& r)
{
    memcpy(data, r.data, 9*sizeof(double) );
}

KDLRotation2::KDLRotation2(::KDL::Rotation2 const& r)
    : s( sin(r.GetRot()) )
    , c( cos(r.GetRot()) )
{
}

KDLFrame::KDLFrame(::KDL::Frame const& f)
    : p(f.p)
    , M(f.M)
{}

KDLFrame2::KDLFrame2(::KDL::Frame2 const& f)
    : p(f.p)
    , M(f.M)
{}

KDLTwist::KDLTwist(::KDL::Twist const& t)
   : vel(t.vel)
   , rot(t.rot)
{} 

KDLWrench::KDLWrench(::KDL::Wrench const& t)
    : force(t.force)
    , torque(t.torque)
{}

KDLJoint::KDLJoint(::KDL::Joint const& j)
    : name(j.getName())
    , type( static_cast<KDLJointType>(j.getType()) )
    , axis( j.JointAxis() )
    , origin( j.JointOrigin() )
{
}

KDLRotationalInertia::KDLRotationalInertia(::KDL::RotationalInertia const& i)
{
    memcpy(data, i.data, 9*sizeof(double));
}

KDLRigidBodyInertia::KDLRigidBodyInertia(::KDL::RigidBodyInertia const& i)
    : m(i.getMass())
    , I(i.getRotationalInertia())
    , h(i.getMass()*i.getCOG())
{}

KDLSegment::KDLSegment(::KDL::Segment const& s)
    : name(s.getName())
    , joint(s.getJoint())
    , I(s.getInertia())
    
{
    f_tip = s.getJoint().pose(0).Inverse()*s.getFrameToTip();
}

KDLChain::KDLChain(::KDL::Chain const& c)
{
    std::vector< ::KDL::Segment>::const_iterator it = c.segments.begin();
    segments.resize(c.segments.size());
    int i = 0;
    for(; it != c.segments.end(); ++it)
    {
        segments[i++] = ( KDLSegment(*it) );
    }
}

}

