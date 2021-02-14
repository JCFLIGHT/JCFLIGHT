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

//ENDEREÇO DE VERIFICAÇÃO DO PRIMEIRO UPLOAD DO FIRMWARE
#define FIRST_LINK_ADDR 1500

//ENDEREÇOS PARA A LISTA COMPLETA DE PARAMETROS
#define KP_ACC_AHRS_ADDR 0
#define KI_ACC_AHRS_ADDR 1
#define KP_MAG_AHRS_ADDR 2
#define KI_MAG_AHRS_ADDR 3
/*
#define SERVO_PULSE_MIN_ADDR 4
#define SERVO1_PULSE_MIDDLE_ADDR 6
#define SERVO_PULSE_MAX_ADDR 8
//#define SERVOS_LPF_ADDR 10
*/
#define AL_AHRS_BA_ADDR 12
#define AL_IMU_BA_ADDR 13
#define AL_IMU_SWING_ADDR 15
#define AL_TRIGGER_MOTOR_DELAY_ADDR 16
#define AL_ELEVATOR_ADDR 18
#define AL_SPINUP_ADDR 19
#define AL_SPINUP_TIME_ADDR 21
#define AL_MAX_THROTTLE_ADDR 23
#define AL_EXIT_ADDR 25
#define AL_ALTITUDE_ADDR 27
#define BATT_VOLTAGE_FACTOR_ADDR 28
#define BATT_AMPS_VOLT_ADDR 32
#define BATT_AMPS_OFFSET_ADDR 34
#define CC_BANKANGLE_ADDR 36
#define CC_TIME_ADDR 37
//#define FAILSAFE_ADDR 38
#define GIMBAL_MIN_ADDR 40
#define GIMBAL_MID_ADDR 42
#define GIMBAL_MAX_ADDR 44
#define LAND_CHECKACC_ADDR 46
#define LAND_LPF_ADDR 47
/*
#define SERVO2_PULSE_MIDDLE_ADDR 48
#define SERVO3_PULSE_MIDDLE_ADDR 50
#define SERVO4_PULSE_MIDDLE_ADDR 52
*/
//53 LIVRE
#define AUTODISARM_ADDR 54
#define AUTODISARM_THR_MIN_ADDR 55
#define AUTODISARM_YPR_MIN_ADDR 57
#define AUTODISARM_YPR_MAX_ADDR 59
//#define NEARNESS_ADDR 61
#define WHEELS_ADDR 62
#define GPS_BAUDRATE_ADDR 63
#define NAV_VEL_ADDR 64
#define WP_RADIUS_ADDR 66
#define RTH_LAND_ADDR 67
#define GPS_TILT_COMP_ADDR 68
//#define AIRSPEED_TYPE_ADDR 69
#define AIRSPEED_SAMPLES_ADDR 70
#define AIRSPEED_FACTOR_ADDR 71
#define ROLL_ADJ_ADDR 73
#define PITCH_ADJ_ADDR 75
#define YAW_ADJ_ADDR 77
#define THROTTLE_FACTOR_ADDR 79

//ENDEREÇOS PARA AS CONFIGURAÇÕES BASICAS E MÉDIAS
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
//#define MOTORSPEED_ADDR 512
#define AUTOMISSION_ADDR 513
#define AUTOLAND_ADDR 514
#define ARMDISARM_ADDR 515
#define DISP_PASSIVES_ADDR 516
#define DECLINATION_ADDR 517
#define UART_NUMB_2_ADDR 521
#define COMPASS_ROTATION_ADDR 522
#define COMPASS_TYPE_ADDR 523
#define RTH_ALTITUDE_ADDR 524
#define UART_NUMB_3_ADDR 525
#define THROTTLE_DZ_ADDR 526
#define YAW_DZ_ADDR 527
#define PITCH_DZ_ADDR 528
#define ROLL_DZ_ADDR 529
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
#define ACC_ROLL_ADJUST_ADDR 560
#define ACC_PITCH_ADJUST_ADDR 562
#define ACC_YAW_ADJUST_ADDR 564
#define RC_PULSE_MIN_ADDR 566
#define RC_PULSE_MAX_ADDR 568
#define THROTTLE_MIDDLE_ADDR 570
#define THROTTLE_EXPO_ADDR 571
#define RC_RATE_ADDR 572
#define RC_EXPO_ADDR 573
#define ROLL_RATE_ADDR 574
#define PITCH_RATE_ADDR 575
#define YAW_RATE_ADDR 576
//#define AH_THROTTLE_ROVER_ADDR 577
#define AH_DEADZONE_ADDR 579
#define AH_SAFE_ALT_ADDR 580
#define AH_MIN_VEL_VERT_ADDR 581
#define FAILSAFE_VAL_ADDR 588
#define THROTTLE_MIN_ADDR 590
#define YAW_MIN_ADDR 592
#define PITCH_MIN_ADDR 594
#define ROLL_MIN_ADDR 596
#define THROTTLE_MAX_ADDR 598
#define YAW_MAX_ADDR 600
#define PITCH_MAX_ADDR 602
#define ROLL_MAX_ADDR 604
#define CH_REVERSE_ADDR 606
#define SERVO1_MIN_ADDR 607
#define SERVO2_MIN_ADDR 609
#define SERVO3_MIN_ADDR 611
#define SERVO4_MIN_ADDR 613
#define SERVO1_MID_ADDR 615
#define SERVO2_MID_ADDR 617
#define SERVO3_MID_ADDR 619
#define SERVO4_MID_ADDR 621
#define SERVO1_MAX_ADDR 623
#define SERVO2_MAX_ADDR 625
#define SERVO3_MAX_ADDR 627
#define SERVO4_MAX_ADDR 629
#define SERVO1_RATE_ADDR 582
#define SERVO2_RATE_ADDR 584
#define SERVO3_RATE_ADDR 586
#define SERVO4_RATE_ADDR 631
#define SERVOS_REVERSE_ADDR 633
#define SERVOS_LPF_ADDR 634
#define AIRSPEED_TYPE_ADDR 635
#endif