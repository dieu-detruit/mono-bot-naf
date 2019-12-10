#pragma once

#include <array>

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif


/* @description: ユーザーのコントロール関数
 * @param: state 足首と膝の角度
 * @return: 足首と膝のトルク入力
 */
std::array<dReal, 2> user_callback(const std::array<dReal, 2>& state);

void loadParams();
