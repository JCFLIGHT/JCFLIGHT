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

/*
 FUTURAMENTE UM ARQUIVO EM PYTHON SERÁ UMA BOA SACADA PARA AUTOMATIZAR ESSA EXTENSÃO,ASSIM GERANDO OS ENDEREÇOS AUTOMATICAMENTE.

 POR EXEMPLO:
 O DESENVOLVEDOR ENTRA COM O NOME DO ENDEREÇO A SER GERADO E O TAMANHO QUE ELE OCUPA.

 DESSA FORMA:
 (KP_ACC_AHRS_ADDR, sizeof(uint8_t)) -> OCUPA O ENDEREÇO 0
 //O PROXIMO ENDEREÇO SERÁ 1 POR QUE O ANTERIOR CONSUMIU APENAS 1 BYTE
 (KI_ACC_AHRS_ADDR, sizeof(uint8_t)) -> OCUPA O ENDEREÇO 1
 //O PROXIMO ENDEREÇO SERÁ 2 POR QUE O ANTERIOR CONSUMIU APENAS 1 BYTE
 //E ASSIM POR DIANTE...
 //O "sizeof" É QUEM VAI DITAR O PROMIXO ADDR DA EEPROM A SER USADO

 E O ARQUIVO EM PYTHON GERA AUTOMATICAMENTE O #define DO ENDEREÇO
 #define KP_ACC_AHRS_ADDR 0
 #define KI_ACC_AHRS_ADDR 1
*/

//ENDEREÇO DE VERIFICAÇÃO DO PRIMEIRO UPLOAD DO FIRMWARE
#define FIRMWARE_FIRST_USAGE_ADDR 1500

//ENDEREÇOS PARA A LISTA DE PARAMETROS
#define KP_ACC_AHRS_ADDR 0
#define KI_ACC_AHRS_ADDR 1
#define KP_MAG_AHRS_ADDR 2
#define KI_MAG_AHRS_ADDR 3
#define AL_AHRS_BA_ADDR 4
#define AL_IMU_BA_ADDR 6
#define AL_IMU_GPS_VEL_ADDR 7
#define AL_TRIGGER_MOTOR_DELAY_ADDR 8
#define AL_ELEVATOR_ADDR 10
#define AL_SPINUP_ADDR 11
#define AL_SPINUP_TIME_ADDR 13
#define AL_MAX_THROTTLE_ADDR 15
#define AL_EXIT_ADDR 17
#define AL_ALTITUDE_ADDR 19
#define BATT_VOLTAGE_FACTOR_ADDR 23
#define BATT_AMPS_VOLT_ADDR 27
#define BATT_AMPS_OFFSET_ADDR 31
#define CC_BANKANGLE_ADDR 35
#define CC_TIME_ADDR 36
#define GIMBAL_MIN_ADDR 37
#define GIMBAL_MID_ADDR 39
#define GIMBAL_MAX_ADDR 41
#define LAND_CHECKACC_ADDR 43
#define LAND_LPF_ADDR 44
#define THROTTLE_FACTOR_ADDR 45
#define AUTODISARM_ADDR 49
#define AUTODISARM_THR_MIN_ADDR 51
#define AUTODISARM_YPR_MIN_ADDR 53
#define AUTODISARM_YPR_MAX_ADDR 55
#define WHEELS_ADDR 57
#define GPS_BAUDRATE_ADDR 58
#define NAV_VEL_ADDR 59
#define WP_RADIUS_ADDR 61
#define RTH_LAND_ADDR 62
#define GPS_TILT_COMP_ADDR 63
#define AIRSPEED_SAMPLES_ADDR 64
#define AIRSPEED_FACTOR_ADDR 65
#define ARM_TIME_SAFETY_ADDR 69
#define DISARM_TIME_SAFETY_ADDR 70
#define COMPASS_CAL_TIME_ADDR 71
#define AUTO_PILOT_MODE_ADDR 72
#define AS_AUTO_CAL_SCALE_ADDR 73 //PROXIMO ADDR = 74

//ENDEREÇOS PARA AS CONFIGURAÇÕES BASICAS E MÉDIAS
#define ACC_ROLL_OFFSET_ADDR 480
#define ACC_PITCH_OFFSET_ADDR 482
#define ACC_YAW_OFFSET_ADDR 484
#define ACC_ROLL_SCALE_ADDR 486
#define ACC_PITCH_SCALE_ADDR 488
#define ACC_YAW_SCALE_ADDR 490
#define MAG_PITCH_OFFSET_ADDR 492
#define MAG_ROLL_OFFSET_ADDR 494
#define MAG_YAW_OFFSET_ADDR 496
#define MAG_ROLL_GAIN_ADDR 560
#define MAG_PITCH_GAIN_ADDR 562
#define MAG_YAW_GAIN_ADDR 564
#define SIMPLE_ADDR 500
#define ALT_HOLD_ADDR 501
#define GPS_HOLD_ADDR 502
#define RTH_ADDR 503
#define STABLIZE_ADDR 504
#define ATACK_ADDR 505
#define PARACHUTE_ADDR 506
#define AUTOFLIP_ADDR 507
#define GIMBAL_ADDR 508
#define FRAME_TYPE_ADDR 509
#define RC_SEQUENCY_ADDR 510
#define FF_OR_CD_ROLL_ADDR 512
#define AUTOMISSION_ADDR 513
#define AUTOLAND_ADDR 514
#define ARMDISARM_ADDR 515
#define DISP_PASSIVES_ADDR 516
#define MAG_DECLINATION_ADDR 517
#define UART_NUMB_1_ADDR 521
#define UART_NUMB_2_ADDR 522
#define UART_NUMB_3_ADDR 523
#define COMPASS_ROTATION_ADDR 524
#define RTH_ALTITUDE_ADDR 525
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
#define KP_VEL_Z_ADDR 539
#define KI_VEL_Z_ADDR 579
#define KD_VEL_Z_ADDR 580
#define KP_GPSPOS_ADDR 540
#define KI_GPSPOS_ADDR 541
#define TPA_PERCENT_ADDR 542
#define BREAKPOINT_ADDR 543
#define HW_GYRO_LPF_ADDR 545
#define DERIVATIVE_LPF_ADDR 546
#define RC_LPF_ADDR 548
#define KALMAN_ADDR 550
#define BI_ACC_LPF_ADDR 551
#define BI_GYRO_LPF_ADDR 553
#define BI_ACC_NOTCH_ADDR 555
#define BI_GYRO_NOTCH_ADDR 557
#define MOT_COMP_STATE_ADDR 559
#define THR_ATTITUDE_MIN_ADDR 566
#define THR_ATTITUDE_MAX_ADDR 568
#define THROTTLE_MIDDLE_ADDR 570
#define THROTTLE_EXPO_ADDR 571
#define RC_RATE_ADDR 572
#define RC_EXPO_ADDR 573
#define ROLL_BANK_ADDR 574
#define PITCH_BANK_MIN_ADDR 575
#define YAW_RATE_ADDR 576
#define FF_OR_CD_PITCH_ADDR 577
#define FF_OR_CD_YAW_ADDR 578
#define SERVO1_RATE_ADDR 582
#define SERVO2_RATE_ADDR 584
#define SERVO3_RATE_ADDR 586
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
#define SERVO4_RATE_ADDR 631
#define SERVOS_REVERSE_ADDR 633
#define SERVOS_LPF_ADDR 634
#define KP_AUTOLEVEL_ADDR 636
#define KI_AUTOLEVEL_ADDR 637
#define KP_HEADING_HOLD_ADDR 638
#define HEADING_HOLD_RATE_LIMIT_ADDR 639
#define PITCH_BANK_MAX_ADDR 640
#define ATTACK_BANK_ADDR 641
#define GPS_BANK_ADDR 642
#define MAX_PITCH_LEVEL_ADDR 643
#define MAX_ROLL_LEVEL_ADDR 644
#define AIRSPEED_TYPE_ADDR 645
#define INTEGRAL_RELAX_LPF_ADDR 646
#define KCD_OR_FF_LPF_ADDR 648
#define PITCH_LEVEL_TRIM_ADDR 650
#define KP_POS_Z_ADDR 581
#define KI_POS_Z_ADDR 652
#define KD_POS_Z_ADDR 653
#define KP_POS_RATE_ADDR 654
#define KI_POS_RATE_ADDR 655
#define KD_POS_RATE_ADDR 656
#define KP_NAV_RATE_ADDR 657
#define KI_NAV_RATE_ADDR 658
#define KD_NAV_RATE_ADDR 659 //PROXIMO ADDR = 660

//OS ENDEREÇOS DE 704 ATÉ O 813 SÃO CONSUMIDOS PELO MODO WAYPOINT
#endif