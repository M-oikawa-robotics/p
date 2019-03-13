/**
 * \file
 * \author Kutsuzawa, Kyo
 *
 * \brief 6軸マニピュレータの制御インタフェース
 *
 * 制御系のブロック線図で"ROBOT"などと置かれるブロックに相当する機能と，
 * 制御系で用いられるマニピュレータのパラメータを提供する。
 *
 * シミュレーション環境と実機環境とで同一のインタフェースが利用できるように保守される。
 */

#ifndef ROBOT_INCLUDED_BDEDA264_74D6_4EA0_8406_1FE9371B03C4
#define ROBOT_INCLUDED_BDEDA264_74D6_4EA0_8406_1FE9371B03C4

/**
 * \brief 各リンクの長さ[m]
 */
extern double link_length[7];
/**
 * \brief 各リンクの慣性モーメント[kgm^2]
 */
extern double link_inertia[6];
/**
 * \brief 各リンクの質量[kg]
 */
extern double link_weight[7];

/**
 * \brief マニピュレータの関節角度を取得する
 *
 * \param [out] joint_angle 関節角度[rad]が収まる。
 */
void robot_get_joint_angle(double joint_angle[6]);
/**
 * \brief 各関節にトルクを指令する
 *
 * \param [in] joint_torque 各関節の出力するトルク[Nm]
 */

void robot_get_force(double force[6]);
//void robot_set_LSTM_output(const std::vector<double> LSTM_output);
void robot_get_LSTM_output(double LSTM_output[12]);
int robot_set_joint_torque(const double joint_torque[6]);

/**
 * \brief 関節角度から同時変換行列を求める
 *
 * 手先座標系での成分表示をワールド座標系での成分表示に変換する座標変換行列を求める。
 *
 * \param [out] T 同時変換行列が収まる
 * \param [in] joint_angle 関節角度
 */
void robot_get_transformation(double T[4][4], const double joint_angle[6]);

#endif // SERIALROBOT_INCLUDED_BDEDA264_74D6_4EA0_8406_1FE9371B03C4
