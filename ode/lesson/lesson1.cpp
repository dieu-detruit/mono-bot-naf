/*** このプログラムについて
　最低限必要な要素のみ。まずは関数同士の入れ子構造を把握する程度で。
　主に書き換えたいのは、生成と描画。最初にボディ（運動用）と、ジオメトリ（衝突用）を生成し、
　毎ステップごとに運動計算された位置と姿勢を取得して、その位置に描画するという流れ。　***/

#include <drawstuff/drawstuff.h>  // 描画用ヘッダーファイル
#include <ode/ode.h>              // ODE用ヘッダーファイル

#include "texturepath.h"

/*** 単精度と倍精度の両方に対応 ***/
#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD  // 球
#endif

dWorldID world;              // 動力学計算用ワールド
dSpaceID space;              // 衝突検出用スペース
dGeomID ground;              // 地面
dJointGroupID contactgroup;  // コンタクトグループ
dsFunctions fn;              // ドロースタッフ用の構造体

/*** MyObject構造体 ***/
struct MyObject {
    dBodyID body;   // 剛体のID番号
    dGeomID geom;   // ジオメトリのID番号
    dReal l, r, m;  // 長さ、半径、質量
};

MyObject ball[2];
dJointID s_joint;

/*** 衝突検出 コールバック関数 ***/  // 衝突が起こりそうなときにだけ発動する
static void nearCallback(void* data, dGeomID o1, dGeomID o2)
{
    static const int N = 10;  // 接触点数の最大値
    dContact contact[N];      // 接触点

    // 2つのボディがジョイントで結合されていたら衝突検出しない
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
        return;
    int isGround = ((ground == o1) || (ground == o2));  // 接触のどちらかが地面ならisGroundに非0を代入

    /** 衝突情報の生成 **/
    int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));  // 衝突点数
    if (isGround) {
        for (int i = 0; i < n; i++) {
            contact[i].surface.soft_erp = 0.2;                                                    // 接触点のERP
            contact[i].surface.soft_cfm = 0.001;                                                  // 接触点のCFM
            contact[i].surface.mu = dInfinity;                                                    // 摩擦係数
            contact[i].surface.mode = dContactBounce;                                             // 接触面の反発性を設定
            contact[i].surface.bounce = 0.9;                                                      // 反発係数
            contact[i].surface.bounce_vel = 0.0;                                                  // 反発最低速度
            dJointID c = (dJointCreateContact(world, contactgroup, &contact[i]));                 // 接触ジョイントの生成
            dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));  // 接触している2つの剛体を接触ジョイントにより拘束
        }
    }
}

/*** 描画 ***/
static void drawObjects()
{
    dReal h = 0.0001;  //step_size (dWorldStepで定義したものと同じ);
    dReal kp = 500;    // ばね定数 (spring constant)
    dReal kd = 1.0;    // 減衰定数 (damping constant)
    dReal erp = h * kp / (h * kp + kd);
    dReal cfm = 1.0 / (h * kp + kd);
    dJointSetSliderParam(s_joint, dParamLoStop, 0.0);  // ばねの自然長位置
    dJointSetSliderParam(s_joint, dParamHiStop, 0.0);  // ばねの自然長位置
    dJointSetSliderParam(s_joint, dParamStopERP, erp);
    dJointSetSliderParam(s_joint, dParamStopCFM, cfm);

    for (int i = 0; i < 2; ++i) {
        dsSetColor(1.0, 0.0, 0.0);                          // 色(RGB)
        const dReal* pos = dBodyGetPosition(ball[i].body);  // 位置を取得
        const dReal* rot = dBodyGetRotation(ball[i].body);  // 姿勢を取得
        dsDrawSphereD(pos, rot, ball[i].r);                 // 球の描画
    }
}

/***  シミュレーションループ　***/
static void simLoop(int pause)
{
    dSpaceCollide(space, 0, &nearCallback);  // 衝突検出
    dWorldStep(world, 0.01);                 // シミュレーションを1ステップ進める
    dJointGroupEmpty(contactgroup);          // ジョイントグループを空にする
    drawObjects();                           // 物体の描画
}

/*** 生成 ***/
const dReal r[] = {0.2, 0.5};
const dReal m[] = {1.2, 2.5};
const dReal xinit[] = {0.0, 0.5};
const dReal yinit[] = {-0.2, 0.5};
const dReal zinit[] = {2.2, 3.0};

static void makeObjects()
{
    for (int i = 0; i < 2; ++i) {
        ball[i].r = r[i];
        ball[i].m = m[i];
        dMass mass;

        ball[i].body = dBodyCreate(world);
        dMassSetZero(&mass);
        dMassSetSphereTotal(&mass, ball[i].m, ball[i].r);  // ボディの生成
        dBodySetMass(ball[i].body, &mass);
        dBodySetPosition(ball[i].body, xinit[i], yinit[i], zinit[i]);  // 初期位置
        ball[i].geom = dCreateSphere(space, ball[i].r);                // ジオメトリの生成
        dGeomSetBody(ball[i].geom, ball[i].body);                      // ボディとジオメトリの関連付け
    }


    // スライダージョイント
    s_joint = dJointCreateSlider(world, 0);
    dJointAttach(s_joint, ball[0].body, ball[1].body);
    dJointSetSliderAxis(s_joint, 0, 0, 1);
    dJointSetSliderParam(s_joint, dParamLoStop, -0.25);
    dJointSetSliderParam(s_joint, dParamHiStop, 0.25);
}

/*** 前処理　カメラ配置など　***/
void start()
{
    static float xyz[3] = {3.0, 0.0, 1.0};  // 視点の位置
    static float hpr[3] = {-180, 0, 0};     // 視線の方向（パン、チルト、ロール）
    dsSetViewpoint(xyz, hpr);               // カメラの設定
    dsSetSphereQuality(3);                  // 球の品質設定
}

/*** 描画関数の設定 ***/
void setDrawStuff()
{
    fn.version = DS_VERSION;                       // ドロースタッフのバージョン
    fn.start = &start;                             // 前処理 start関数のポインタ
    fn.step = &simLoop;                            // simLoop関数のポインタ
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;  // テクスチャ
}

/*** main関数 ***/
int main(int argc, char* argv[])
{
    setDrawStuff();                            // 描画関数の設定
    dInitODE();                                // ODEの初期化
    world = dWorldCreate();                    // 世界の創造
    dWorldSetGravity(world, 0, 0, -9.81);      // 重力設定
    space = dHashSpaceCreate(0);               // 衝突用空間の創造
    contactgroup = dJointGroupCreate(0);       // ジョイントグループの生成
    ground = dCreatePlane(space, 0, 0, 1, 0);  // 平面ジオメトリの生成

    makeObjects();                                // 物体の生成
    dsSimulationLoop(argc, argv, 640, 480, &fn);  // シミュレーションループ

    dJointDestroy(s_joint);
    dSpaceDestroy(space);  // 空間の終焉
    dWorldDestroy(world);  // 世界の終焉
    dCloseODE();           // ODEの終了
    return 0;
}
