/**
 * \file
 * \author Kutsuzawa Kyo
 *
 * \brief シリアルリンクマニピュレータの制御を行う
 *
 */
#ifndef CONTROL_INCLUDED_2F94393E_7A60_402C_BE90_7F3130BE0B76
#define CONTROL_INCLUDED_2F94393E_7A60_402C_BE90_7F3130BE0B76

/** \brief 実機の環境なら1，ODEシミュレーションなら0 */
#define ACTUALROBOT_ENVIRONMENT 0

void control_joint(void);
void control_turnover(void);
void control_sliding(void);

#endif  // CONTROL_INCLUDED_2F94393E_7A60_402C_BE90_7F3130BE0B76
