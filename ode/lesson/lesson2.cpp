/*** このプログラムについて
　衝突対象が地面のみでなくなり、斜面が加わった。衝突検出を書き換えなくてはならない。
　斜面は動かないのでボディは必要なく、ジオメトリのみ生成。また、回転行列による姿勢の設定を行う。
　細かい変更として、標準キーコマンドの一時停止を使うためsimLoopを書き換えた。　　　　　　　　　***/

/**　標準キーコマンド
　Ctrl+t　テクスチャのON/OFF
　Ctrl+s　影のON/OFF
　Ctrl+x　強制終了
　Ctrl+p　一時停止/再開
　Ctrl+o　コマ送り　　　　　　**/

#include "texturepath.h"
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#ifdef _MSC_VER
#pragma warning(disable : 4244 4305)  // for VC++, no precision loss complaints
#endif

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

static dWorldID world;
static dSpaceID space;
static dGeomID ground;
static dGeomID slope;
static dJointGroupID contactgroup;
dsFunctions fn;

typedef struct {
    dBodyID body;
    dGeomID geom;
    dReal r, m;
} MyObject;
MyObject ball;

static void nearCallback(void* data, dGeomID o1, dGeomID o2)
{
    const int N = 10;
    dContact contact[N];

    /** 相手が地面か斜面なら衝突
  　じつは、常にisGround=1としてもプログラムは動くが、参考書では推奨されない**/
    int isGround = ((ground == o1 || slope == o1) || (ground == o2 || slope == o2));

    int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
    if (isGround) {
        for (int i = 0; i < n; i++) {
            contact[i].surface.mode = dContactSoftERP | dContactSoftCFM;
            contact[i].surface.soft_erp = 1.0;  // 接触点のERP
            contact[i].surface.soft_cfm = 0.0;  // 接触点のCFM
            contact[i].surface.mu = dInfinity;  // 摩擦係数
            dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
            dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
        }
    }
}

static void drawObjects()
{
    const dReal *pos, *R;

    dsSetColor(1.0, 0.0, 0.0);
    pos = dBodyGetPosition(ball.body);
    R = dBodyGetRotation(ball.body);
    dsDrawSphere(pos, R, ball.r);

    dsSetColor(1.2, 1.2, 0.0);
    pos = dGeomGetPosition(slope);
    R = dGeomGetRotation(slope);
    dVector3 sides;
    dGeomBoxGetLengths(slope, sides);
    dsDrawBox(pos, R, sides);
}

static void simLoop(int pause)
{
    fn.path_to_textures = "../../drawstuff/textures";

    /** 一時停止 **/  //Ctrl+pでpauseの値が0と1で交互に変わる
    if (!pause) {
        dSpaceCollide(space, 0, &nearCallback);
        dWorldStep(world, 0.01);
        dJointGroupEmpty(contactgroup);
    }

    drawObjects();
}

void start()
{
    static float xyz[3] = {3.0, 0.0, 1.0};
    static float hpr[3] = {-180.0, 0.0, 0.0};
    dsSetViewpoint(xyz, hpr);
    dsSetSphereQuality(3);
}

void setDrawStuff()
{
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = NULL;
    fn.stop = NULL;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
}

void makeObjects()
{
    /** 球の生成 **/
    dMass m1;
    dReal x0 = -10.0, y0 = 0.0, z0 = 2.2;

    ball.r = 0.2;
    ball.m = 1.0;
    ball.body = dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetSphereTotal(&m1, ball.m, ball.r);
    dBodySetMass(ball.body, &m1);
    dBodySetPosition(ball.body, x0, y0, z0);

    ball.geom = dCreateSphere(space, ball.r);
    dGeomSetBody(ball.geom, ball.body);

    /** 斜面の生成（ジオメトリのみ） **/
    dReal lx = 100, ly = 2, lz = 0.01;  // 絶対座標系x,y,z軸に沿った長さ
    dReal x = 0, y = 0, z = 0;          // 重心の座標
    dMatrix3 R;                         // 回転行列
    dReal ax = 0, ay = 1, az = 0;       // 回転軸ベクトル
    dReal angle = 10.0 * M_PI / 180.0;  // 回転角

    slope = dCreateBox(space, lx, ly, lz);  // 直方体ジオメトリの生成 　slopeはGeomIDと宣言済

    dGeomSetPosition(slope, x, y, z);          // 位置の設定
    dRFromAxisAndAngle(R, ax, ay, az, angle);  // 回転軸周りにangle[rad]の回転行列を計算し行列Rに代入
    dGeomSetRotation(slope, R);                // 姿勢の設定
}

int main(int argc, char* argv[])
{
    dInitODE();
    setDrawStuff();

    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    ground = dCreatePlane(space, 0, 0, 1, 0);
    contactgroup = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, 0, -9.81);

    makeObjects();
    dsSimulationLoop(argc, argv, 640, 480, &fn);

    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
