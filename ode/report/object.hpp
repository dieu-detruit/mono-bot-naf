#pragma once

#include "simulation.hpp"

#include <Eigen/Core>

struct Color {
    double r;
    double g;
    double b;
};

class ObjectBase
{
protected:
    dBodyID body_id;  // ボディ(剛体)のID番号（動力学計算用）
    dGeomID geom_id;  // ジオメトリのID番号(衝突検出計算用）
    double mass;

public:
    ObjectBase()
    {
    }

    ~ObjectBase()
    {
        dBodyDestroy(body_id);
        dGeomDestroy(geom_id);
    }

    virtual void draw() = 0;

    virtual dBodyID getBody() { return body_id; };
    virtual dGeomID getGeometry() { return geom_id; };
};

class Capsule : public ObjectBase
{
    int direction;
    dReal total_mass;
    double radius, length;

    Color color;

public:
    Capsule(
        const int direction,
        const dReal total_mass,
        const float radius, const float length,
        const Color& color)
        : direction(direction), total_mass(total_mass), radius(radius), length(length), color(color)
    {
    }

    void init(const dWorldID world, const dSpaceID space,
        const Eigen::Vector3d& position)
    {
        body_id = dBodyCreate(world);
        dMass mass;
        dMassSetZero(&mass);
        dMassSetCapsuleTotal(&mass, total_mass, direction, radius, length);
        dBodySetMass(body_id, &mass);
        dBodySetPosition(body_id, position(0), position(1), position(2));
        geom_id = dCreateCapsule(space, radius, length);
        dGeomSetBody(geom_id, body_id);
    }

    void draw() override
    {
        dsSetColor(color.r, color.g, color.b);
        const dReal* pos3 = dBodyGetPosition(body_id);
        const dReal* rot3 = dBodyGetRotation(body_id);
        dsDrawCapsule(pos3, rot3, length, radius);
    }
};

class Box : public ObjectBase
{
    dReal total_mass;
    Eigen::Vector3d length;
    Color color;

public:
    Box(
        const dReal total_mass,
        const Eigen::Vector3d& length,
        const Color& color)
        : total_mass(total_mass), length(length), color(color)
    {
    }

    void init(const dWorldID world, const dSpaceID space,
        const Eigen::Vector3d& position)
    {
        body_id = dBodyCreate(world);
        dMass mass;
        dMassSetZero(&mass);
        dMassSetBoxTotal(&mass, total_mass, length(0), length(1), length(2));
        dBodySetMass(body_id, &mass);
        dBodySetPosition(body_id, position(0), position(1), position(2));
        geom_id = dCreateBox(space, length(0), length(1), length(2));
        dGeomSetBody(geom_id, body_id);
    }

    void draw() override
    {
        dsSetColor(color.r, color.g, color.b);
        const dReal* pos3 = dBodyGetPosition(body_id);
        const dReal* rot3 = dBodyGetRotation(body_id);
        const double sides[] = {length(0), length(1), length(2)};
        dsDrawBox(pos3, rot3, sides);
    }
};

class Sphere : public ObjectBase
{
    dReal total_mass;
    double radius;
    Color color;

public:
    Sphere(
        const dReal total_mass,
        const double radius,
        const Color& color)
        : total_mass(total_mass), radius(radius), color(color)
    {
    }

    void init(const dWorldID world, const dSpaceID space,
        const Eigen::Vector3d& position)
    {
        body_id = dBodyCreate(world);
        dMass mass;
        dMassSetZero(&mass);
        dMassSetSphereTotal(&mass, total_mass, radius);
        dBodySetMass(body_id, &mass);
        dBodySetPosition(body_id, position(0), position(1), position(2));
        geom_id = dCreateSphere(space, radius);
        dGeomSetBody(geom_id, body_id);
    }

    void draw() override
    {
        dsSetColor(color.r, color.g, color.b);
        const dReal* pos3 = dBodyGetPosition(body_id);
        const dReal* rot3 = dBodyGetRotation(body_id);
        dsDrawSphere(pos3, rot3, radius);
    }
};
