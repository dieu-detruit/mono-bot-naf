#pragma once

#include "simulation.hpp"

#include <Eigen/Core>

class JointBase
{
protected:
    dBodyID body1;
    dBodyID body2;

    dJointID joint_id;

public:
    JointBase() {}
    ~JointBase() {}

    dJointID getID() { return joint_id; }

    virtual void setParam(int param, dReal parameter) = 0;
};

class Hinge : public JointBase
{
public:
    Hinge(){};
    ~Hinge(){};

    void init(dWorldID world,
        const dBodyID _body1, const dBodyID _body2,
        Eigen::Vector3d anchor_pos,
        Eigen::Matrix<int, 3, 1> axis)
    {
        body1 = _body1;
        body2 = _body2;
        joint_id = dJointCreateHinge(world, 0);
        dJointAttach(joint_id, body1, body2);
        dJointSetHingeAnchor(joint_id,
            anchor_pos(0), anchor_pos(1), anchor_pos(2));
        dJointSetHingeAxis(joint_id, axis(0), axis(1), axis(2));

        dJointSetHingeParam(joint_id, dParamVel, 0);
        dJointAddHingeTorque(joint_id, 0.0);
    }

    virtual void setParam(int param, dReal value) override
    {
        dJointSetHingeParam(joint_id, param, value);
    }
};

class FixedJoint : public JointBase
{
public:
    FixedJoint(){};
    ~FixedJoint(){};

    void init(dWorldID world,
        const dBodyID _body1, const dBodyID _body2)
    {
        body1 = _body1;
        body2 = _body2;
        joint_id = dJointCreateFixed(world, 0);
        dJointAttach(joint_id, body1, body2);
        dJointSetFixed(joint_id);
    }

    virtual void setParam(int param, dReal value) override
    {
        dJointSetFixedParam(joint_id, param, value);
    }
};
