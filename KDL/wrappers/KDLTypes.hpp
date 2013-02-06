#ifndef KDL_WRAPPERS_KDL_TYPES_HPP
#define KDL_WRAPPERS_KDL_TYPES_HPP

#include <string>
#include <vector>

namespace KDL
{
    class Vector;
    class Vector2;
    class Rotation;
    class Rotation2;
    class Frame;
    class Frame2;
    class Wrench;
    class Twist;

    class Joint;
    class JntArray;

    class RotationalInertia;
    class RigidBodyInertia;
    class Segment;
    class Chain;
}

namespace wrappers
{

    struct KDLVector
    {
        double data[3];

        KDLVector() {}
       
        KDLVector(::KDL::Vector const&);
    };
    
    struct KDLVector2
    {
        double data[2];

        KDLVector2() {}
        
        KDLVector2(::KDL::Vector2 const&);
    };
    
    struct KDLRotation
    {
        double data[9];

        KDLRotation() {}
 
        KDLRotation(::KDL::Rotation const&);
    };
    
    struct KDLRotation2
    {
        // sinus angle
        double s;

        // cosinus angle
        double c;

        KDLRotation2() {}

        KDLRotation2(::KDL::Rotation2 const&);
    };
    
    struct KDLFrame
    {
        // origin of the frame
        KDLVector p;
        
        // rotation of the frame
        KDLRotation M;

        KDLFrame() {}
        
        KDLFrame(::KDL::Frame const&);
    };
    
    struct KDLFrame2
    {
        // origin of the frame
        KDLVector2 p;

        // rotation of the frame
        KDLRotation2 M;

        KDLFrame2() {}

        KDLFrame2(::KDL::Frame2 const&);
    };
    
    struct KDLTwist
    {
        KDLVector vel;
        KDLVector rot;

        KDLTwist() {}
 
        KDLTwist(::KDL::Twist const&);
    };
    
    struct KDLWrench
    {
        KDLVector force;
        KDLVector torque;

        KDLWrench() {}

        KDLWrench(::KDL::Wrench const&);
    };

    enum KDLJointType { RotAxis,RotX,RotY,RotZ,TransAxis,TransX,TransY,TransZ,None};

    struct KDLJoint 
    {

        std::string name;
        KDLJointType type;
//        double scale;
//        double offset;
//        double inertia;
//        double damping;
//        double stiffness;

        // variables for RotAxis joint
        KDLVector axis, origin;

    //    Frame  joint_pose;
    //    double q_previous;

        KDLJoint() {}

        KDLJoint(::KDL::Joint const&);
    };

    struct KDLJntArray
    {
        // workaround using opaque in opqaue
        // Eigen::VectorXd
        std::vector<double> data;
    };

    struct KDLRotationalInertia
    {
        double data[9];

        KDLRotationalInertia() {}
   
        KDLRotationalInertia(::KDL::RotationalInertia const&);
    };

    struct KDLRigidBodyInertia
    {
        double m;
        KDLRotationalInertia I;
        KDLVector h;

        KDLRigidBodyInertia() {}

        KDLRigidBodyInertia(::KDL::RigidBodyInertia const&);
    };

    struct KDLSegment
    {
        std::string name;
        KDLJoint joint;
        KDLRigidBodyInertia I;
        KDLFrame f_tip;

        KDLSegment() {}

        KDLSegment(::KDL::Segment const&);
    };

    struct KDLChain
    {
        std::vector<KDLSegment> segments;

        KDLChain() {}

        KDLChain(::KDL::Chain const&);
    };
    
} // end namespace wrappers
#endif // KDL_WRAPPERS_KDL_TYPES_HPP

