#include <drawstuff/drawstuff.h>  // 描画用ヘッダーファイル
#include <ode/ode.h>              // ODE用ヘッダーファイル

#include "joint.hpp"
#include "object.hpp"
#include "simulation.hpp"

#include <Eigen/Core>


namespace Simulation
{

dWorldID world;              // 動力学計算用ワールド
dSpaceID space;              // 衝突検出用スペース
dGeomID ground;              // 地面
dJointGroupID contactgroup;  // コンタクトグループ

int STEPS = 0;  // シミュレーションのステップ数


Capsule leg{
    3, 5.0, 0.2, 0.5,
    Color{1.0, 0.0, 0.0}};
Capsule thigh{
    3, 3.0, 0.2, 0.5,
    Color{0.0, 0.0, 1.0}};
Box foot{
    7.0, Eigen::Vector3d{0.8, 0.7, 0.2},
    Color{0.0, 1.0, 0.0}};
Sphere head{
    2.0, 0.4,
    Color{1.0, 1.0, 0.0}};

Hinge ankle;
Hinge knee;
FixedJoint neck;


void near_callback(void* data, dGeomID o1, dGeomID o2)
{
    constexpr int N = 3;  // 接触点数
    dContact contact[N];

    // 地面との衝突かどうか
    int isGround = (ground == o1 || ground == o2);

    // 2つのボディがジョイントで結合されていたら衝突検出しない
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) {
        return;
    }

    int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));

    for (int i = 0; i < n; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftERP | dContactSoftCFM;
        contact[i].surface.soft_erp = 0.2;         // 接触点のERP
        contact[i].surface.soft_cfm = 0.001;       // 接触点のCFM
        contact[i].surface.mu = 0.7;               // 摩擦係数
        contact[i].surface.mode = dContactBounce;  // 接触面の反発性を設定
        contact[i].surface.bounce = 0.6;           // 反発係数
        contact[i].surface.bounce_vel = 0.0;       // 反発最低速度
        dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
        dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
    }
}

void control()
{
    std::array<dReal, 2> torque_input
        = user_callback(std::array<dReal, 2>{
            dJointGetHingeAngle(ankle.getID()),
            dJointGetHingeAngle(knee.getID())});

    dJointAddHingeTorque(ankle.getID(), torque_input[0]);
    dJointAddHingeTorque(knee.getID(), torque_input[1]);
}

void step_callback(int pause)
{
    if (!pause) {
        STEPS++;
        control();
        dSpaceCollide(space, 0, &near_callback);
        dWorldStep(world, 0.05);
        dJointGroupEmpty(contactgroup);
    }

    leg.draw();
    thigh.draw();
    foot.draw();
    head.draw();
}

void restart();
float xyz[] = {4.0, 0, 2.0};
float hpr[] = {180, -10, 0};

void command_callback(int cmd)
{
    switch (cmd) {
    case 'k':
        hpr[1] += 5.0;
        dsSetViewpoint(xyz, hpr);
        break;
    case 'j':
        hpr[1] -= 5.0;
        dsSetViewpoint(xyz, hpr);
        break;
    case 'h':
        hpr[0] += 5.0;
        dsSetViewpoint(xyz, hpr);
        break;
    case 'l':
        hpr[0] -= 5.0;
        dsSetViewpoint(xyz, hpr);
        break;
    case 'w':
        xyz[0] += 0.5;
        dsSetViewpoint(xyz, hpr);
        break;
    case 'e':
        xyz[0] -= 0.5;
        dsSetViewpoint(xyz, hpr);
        break;
    case 'r':
        restart();
        break;
    default:
        printf("invalid key input\n");
        break;
    }
}

// 初期設定
void start()
{
    dsSetViewpoint(xyz, hpr);  // 視点，視線の設定
    dsSetSphereQuality(3);     // 球の品質設定
}


void initialize(int argc, char* argv[])
{
    dInitODE();

    // callback for drawstuff
    dsFunctions drawstuff;
    drawstuff.version = DS_VERSION;
    drawstuff.start = &start;
    drawstuff.step = &step_callback;
    drawstuff.command = &command_callback;
    drawstuff.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);

    foot.init(world, space, Eigen::Vector3d{0.2, 0.0, 0.15});
    leg.init(world, space, Eigen::Vector3d{0.0, 0.0, 0.6});
    thigh.init(world, space, Eigen::Vector3d{0.0, 0.0, 1.55});
    head.init(world, space, Eigen::Vector3d{0.0, 0.0, 2.4});

    ankle.init(world,
        leg.getBody(), foot.getBody(),
        Eigen::Vector3d{0.2, 0.0, 0.2},
        Eigen::Matrix<int, 3, 1>{0, 1, 0});

    knee.init(world,
        thigh.getBody(), leg.getBody(),
        Eigen::Vector3d{0.0, 0.0, 1.1},
        Eigen::Matrix<int, 3, 1>{0, 1, 0});

    neck.init(world, head.getBody(), thigh.getBody());

    ankle.setParam(dParamLoStop, -0.7 * M_PI);
    ankle.setParam(dParamHiStop, 0.7 * M_PI);
    knee.setParam(dParamLoStop, -0.7 * M_PI);
    knee.setParam(dParamHiStop, 0.7 * M_PI);


    dWorldSetGravity(world, 0, 0, -9.81);
    dWorldSetERP(world, 0.9);   // ERPの設定
    dWorldSetCFM(world, 1e-4);  // CFMの設定

    ground = dCreatePlane(space, 0, 0, 1, 0);
    dsSimulationLoop(argc, argv, 1280, 720, &drawstuff);
}

void destruct()
{
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
}

void restart()
{
    STEPS = 0;  // ステップ数の初期化

    destruct();
}

}  // namespace Simulation

int main(int argc, char* argv[])
{
    loadParams();

    Simulation::initialize(argc, argv);
    Simulation::destruct();


    return 0;
}
