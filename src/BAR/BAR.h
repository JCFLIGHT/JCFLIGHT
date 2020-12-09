/*
   Este arquivo faz parte da JCFLIGHT.

   JCFLIGHT é um software livre: você pode redistribuí-lo e/ou modificar
   sob os termos da GNU General Public License conforme publicada por
   a Free Software Foundation, seja a versão 3 da Licença, ou
   (à sua escolha) qualquer versão posterior.

  JCFLIGHT é distribuído na esperança de ser útil,
  mas SEM QUALQUER GARANTIA; sem mesmo a garantia implícita de
  COMERCIALIZAÇÃO ou ADEQUAÇÃO A UM DETERMINADO FIM. Veja o
  GNU General Public License para mais detalhes.

   Você deve ter recebido uma cópia da Licença Pública Geral GNU
  junto com a JCFLIGHT. Caso contrário, consulte <http://www.gnu.org/licenses/>.
*/

#ifndef BAR_H_
#define BAR_H_

//BASE ADDRESS REGISTER

#define KP_ACC_AHRS_ADDR 0
#define KI_ACC_AHRS_ADDR 1
#define KP_MAG_AHRS_ADDR 2
#define KI_MAG_AHRS_ADDR 3
#define IGNORE_RATE_ADDR 4
#define IGNORE_SLOPE_ADDR 5
#define SERVO_PULSE_MIN_ADDR 6
#define SERVO_PULSE_MIDDLE_ADDR 8
#define SERVO_PULSE_MAX_ADDR 10
#define SERVO_LPF_ADDR 12
#define AL_AHRS_BA_ADDR 14
#define AL_IMU_BA_ADDR 15
#define AL_MOTOR_SPINUP_TIME_ADDR 17

#define ACC_ROLL_ADDR 480
#define ACC_PITCH_ADDR 482
#define ACC_YAW_ADDR 484
#define ACC_ROLL_SCALE_ADDR 486
#define ACC_PITCH_SCALE_ADDR 488
#define ACC_YAW_SCALE_ADDR 490
#define MAG_PITCH_ADDR 492
#define MAG_ROLL_ADDR 494
#define MAG_YAW_ADDR 496
#define IOC_ADDR 500
#define ALT_HOLD_ADDR 501
#define GPS_HOLD_ADDR 502
#define RTH_ADDR 503
#define STABLIZE_ADDR 504
#define ATACK_ADDR 505
#define PARACHUTE_ADDR 506
#define AUTOFLIP_ADDR 507
#define GIMBAL_ADDR 508
#define FRAMETYPE_ADDR 509
#define RECEIVER_ADDR 510
#define MOTORSPEED_ADDR 512
#define AUTOMISSION_ADDR 513
#define AUTOLAND_ADDR 514
#define ARMDISARM_ADDR 515
#define SAFEBUTTON_ADDR 516
#define DECLINATION_ADDR 517
#define UART2_ADDR 521
#define COMPASS_ROTATION_ADDR 522
#define COMPASS_TYPE_ADDR 523
#define RTH_ALTITUDE_ADDR 524
#define UART3_ADDR 525
#define SERVO1_TRIM_ADDR 526
#define SERVO2_TRIM_ADDR 527
#define SERVO3_TRIM_ADDR 528
#define SERVO4_TRIM_ADDR 529
#define KP_ROLL_ADDR 530
#define KI_ROLL_ADDR 531
#define KD_ROLL_ADDR 532
#define KP_PITCH_ADDR 533
#define KI_PITCH_ADDR 534
#define KD_PITCH_ADDR 535
#define KP_YAW_ADDR 536
#define KI_YAW_ADDR 537
#define KD_YAW_ADDR 538
#define KP_ALTITUDE_ADDR 539
#define KP_GPSPOS_ADDR 540
#define KI_GPSPOS_ADDR 541
#define TPA_PERCENT_ADDR 542
#define BREAKPOINT_ADDR 543
#define GYRO_LPF_ADDR 545
#define DERIVATIVE_LPF_ADDR 546
#define RC_LPF_ADDR 548
#define KALMAN_ADDR 550
#define BI_ACC_LPF_ADDR 551
#define BI_GYRO_LPF_ADDR 553
#define BI_ACC_NOTCH_ADDR 555
#define BI_GYRO_NOTCH_ADDR 557
#define MOTCOMP_STATE_ADDR 559
#endif