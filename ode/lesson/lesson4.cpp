/*** このプログラムについて
　得られる物理量からモータを制御する。シミュレーションループ内を参照。　***/

#include <drawstuff/drawstuff.h>  // 描画用ヘッダーファイル
#include <ode/ode.h>              // ODE用ヘッダーファイル

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

dWorldID world;              // 動力学計算用ワールド
dSpaceID space;              // 衝突検出用スペース
dGeomID ground;              // 地面
dJointGroupID contactgroup;  // コンタクトグループ
dsFunctions fn;

typedef struct {     /** MyObject構造体 **/
    dBodyID body;    // ボディ(剛体)のID番号（動力学計算用）
    dGeomID geom;    // ジオメトリのID番号(衝突検出計算用）
    double l, r, m;  // 長さ[m], 半径[m], 質量[kg]
} MyObject;

static MyObject leg, arm, mot, ball;
static dReal legl[3] = {1.0, 4.0, 25.0};
static dReal arml[3] = {40.0, 4.0, 1.0};
static dJointID h_joint, f1_joint, f2_joint;
static int STEPS = 0;        // シミュレーションのステップ数
static dReal H_ANGLE = 0.0;  // ヒンジ角

/*** 衝突検出 ***/
static void nearCallback(void* data, dGeomID o1, dGeomID o2)
{
    static const int N = 10;  // 接触点数
    dContact contact[N];

    int isGround = ((ground == o1 || ball.geom == o1) || (ground == o2 || ball.geom == o2));

    // 2つのボディがジョイントで結合されていたら衝突検出しない
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
        return;

    int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
    if (isGround) {
        for (int i = 0; i < n; i++) {
            contact[i].surface.mode = dContactBounce | dContactSoftERP | dContactSoftCFM;
            contact[i].surface.soft_erp = 0.2;         // 接触点のERP
            contact[i].surface.soft_cfm = 0.001;       // 接触点のCFM
            contact[i].surface.mu = dInfinity;         // 摩擦係数
            contact[i].surface.mode = dContactBounce;  // 接触面の反発性を設定
            contact[i].surface.bounce = 0.8;           // 反発係数
            contact[i].surface.bounce_vel = 0.0;       // 反発最低速度
            dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
            dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
        }
    }
}

/*** 描画 ***/
static void drawObjects()
{
    /** 脚の描画 **/
    dsSetColor(0.0, 0.6, 0.9);
    const dReal* pos1 = dBodyGetPosition(leg.body);
    const dReal* rot1 = dBodyGetRotation(leg.body);
    dsDrawBoxD(pos1, rot1, legl);

    /** 腕の描画 **/
    dsSetColor(0.0, 1.0, 1.0);
    const dReal* pos2 = dBodyGetPosition(arm.body);
    const dReal* rot2 = dBodyGetRotation(arm.body);
    dsDrawBoxD(pos2, rot2, arml);

    /** モータの描画 **/
    dsSetColor(1.0, 1.0, 0.0);
    const dReal* pos3 = dBodyGetPosition(mot.body);
    const dReal* rot3 = dBodyGetRotation(mot.body);
    dsDrawCapsule(pos3, rot3, mot.l, mot.r);

    /** 球の描画 **/
    dsSetColor(1.0, 0.5, 0.0);
    const dReal* pos4 = dBodyGetPosition(ball.body);
    const dReal* rot4 = dBodyGetRotation(ball.body);
    dsDrawSphere(pos4, rot4, ball.r);
}

/*** ヒンジジョイントの制御 ***/
static void controlHinge(dReal target)
{
    static dReal kp = 10.0, fmax = 2000;

    dReal tmp = dJointGetHingeAngle(h_joint);  // 現在角度
    dReal diff = target - tmp;
    if (diff >= M_PI)
        diff -= M_PI;
    if (diff <= -M_PI)
        diff += M_PI;
    dReal u = kp * diff;  // 操作量

    dJointSetHingeParam(h_joint, dParamVel, u);      // 速度を与える
    dJointSetHingeParam(h_joint, dParamFMax, fmax);  // 駆動力の上限
}

/*** シミュレーションループ ***/
static void simLoop(int pause)
{
    /** ヒンジジョイントの目標角を計算
   球の位置と速度から計算しており、PD制御らしきことをしている **/
    const dReal* bpos = dBodyGetPosition(ball.body);                // 位置を取得
    const dReal* bvel = dBodyGetLinearVel(ball.body);               // 速度を取得
    const dReal bdif = bpos[0] / (0.5 * arml[0]) - 1.0;             // 目標位置から遠いほど絶対値が大きい
    const dReal targ = (bdif * 45.0 + bvel[0] * 1.5) * M_PI / 180;  // 速度の影響も入れる

    if (!pause) {
        STEPS++;
        controlHinge(targ);
        dSpaceCollide(space, 0, &nearCallback);
        dWorldStep(world, 0.05);
        dJointGroupEmpty(contactgroup);
    }
    drawObjects();
}

/*** 生成 ***/
void createObjects()
{
    dMass mass;

    /** 脚 **/
    leg.m = 10.0;
    leg.body = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass, leg.m, legl[0], legl[1], legl[2]);
    dBodySetMass(leg.body, &mass);
    dBodySetPosition(leg.body, 0.0, 0.0, 0.5 * legl[2]);
    leg.geom = dCreateBox(space, legl[0], legl[1], legl[2]);
    dGeomSetBody(leg.geom, leg.body);

    /** 腕 **/
    arm.m = 1.0;
    arm.body = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass, arm.m, arml[0], arml[1], arml[2]);
    dBodySetMass(arm.body, &mass);
    dBodySetPosition(arm.body, 0.5 * arml[0], 0.0, legl[2]);
    arm.geom = dCreateBox(space, arml[0], arml[1], arml[2]);
    dGeomSetBody(arm.geom, arm.body);

    /** モータ **/
    mot.l = 4.0;
    mot.r = 2.0;
    mot.m = 10.0;
    dReal motangle = 90.0 * M_PI / 180.0;
    mot.body = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass, mot.m, 2, mot.r, mot.l);
    dBodySetMass(mot.body, &mass);
    dBodySetPosition(mot.body, 0.0, 0.0, legl[2]);
    dQuaternion q;
    dQFromAxisAndAngle(q, 1, 0, 0, motangle);
    dBodySetQuaternion(mot.body, q);
    mot.geom = dCreateCapsule(space, mot.r, mot.l);
    dGeomSetBody(mot.geom, mot.body);

    /** 球 **/
    ball.r = 2.0;
    ball.m = 1.0;
    ball.body = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetSphereTotal(&mass, ball.m, ball.r);
    dBodySetMass(ball.body, &mass);
    dBodySetPosition(ball.body, 0.3 * arml[0], 0.0, 1.5 * legl[2]);
    ball.geom = dCreateSphere(space, ball.r);
    dGeomSetBody(ball.geom, ball.body);

    /** 固定ジョイント **/  // これは動かない
    f1_joint = dJointCreateFixed(world, 0);
    dJointAttach(f1_joint, leg.body, 0);
    dJointSetFixed(f1_joint);

    /** 固定ジョイント **/
    f2_joint = dJointCreateFixed(world, 0);
    dJointAttach(f2_joint, leg.body, mot.body);
    dJointSetFixed(f2_joint);

    /** ヒンジジョイント **/
    h_joint = dJointCreateHinge(world, 0);
    dJointAttach(h_joint, leg.body, arm.body);
    dJointSetHingeAnchor(h_joint, 0.0, 0.0, legl[2]);
    dJointSetHingeAxis(h_joint, 0, 1, 0);
}

/*** 破壊 ***/
void destroyObjects()
{
    dJointDestroy(h_joint);
    dJointDestroy(f1_joint);
    dJointDestroy(f2_joint);
    dBodyDestroy(leg.body);
    dGeomDestroy(leg.geom);
    dBodyDestroy(arm.body);
    dGeomDestroy(arm.geom);
    dBodyDestroy(mot.body);
    dGeomDestroy(mot.geom);
    dBodyDestroy(ball.body);
    dGeomDestroy(ball.geom);
}

/*** シミュレーションの再スタート ***/
static void restart()
{
    STEPS = 0;      // ステップ数の初期化
                    // S_LENGTH = 0.0;    // スライダ長の初期化
    H_ANGLE = 0.0;  // ヒンジ角度の初期化

    destroyObjects();                     // ロボットの破壊
    dJointGroupDestroy(contactgroup);     // ジョイントグループの破壊
    contactgroup = dJointGroupCreate(0);  // ジョイントグループの生成
    createObjects();                      // ロボットの生成
}

/*** キー操作 ***/
static void command(int cmd)
{
    switch (cmd) {
    case 'u':
        dBodyAddForce(ball.body, 0, 0, 20.0 * ball.m);
        break;
    case 'y':
        dBodyAddForce(ball.body, 3, 0, 9.81 * ball.m);
        break;
    case 'i':
        dBodyAddForce(ball.body, -3, 0, 9.81 * ball.m);
        break;
    case 'r':
        restart();
        break;
    default:
        printf("key missed \n");
        break;
    }
}

static void start() /*** 初期設定 ***/
{
    dsSetViewpoint(xyz, hpr);  // 視点，視線の設定
    dsSetSphereQuality(3);     // 球の品質設定
}

void setDrawStuff() /*** 描画関数の設定 ***/
{
    fn.version = DS_VERSION;                       // ドロースタッフのバージョン
    fn.start = &start;                             // 前処理 start関数のポインタ
    fn.step = &simLoop;                            // simLoop関数のポインタ
    fn.command = &command;                         // キー入力関数へのポインタ
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;  // テクスチャ
}

/*** メイン関数 ***/
int main(int argc, char* argv[])
{
    dInitODE();
    setDrawStuff();

    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);

    dWorldSetGravity(world, 0, 0, -9.81);
    dWorldSetERP(world, 0.9);   // ERPの設定
    dWorldSetCFM(world, 1e-4);  // CFMの設定
    ground = dCreatePlane(space, 0, 0, 1, 0);
    createObjects();
    dsSimulationLoop(argc, argv, 1280, 720, &fn);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
