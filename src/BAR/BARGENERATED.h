﻿/* 
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

#pragma once

/*
BAR - BASE ADDRESS REGISTER

ESSE ARQUIVO HEADER FOI GERADO AUTOMATICAMENTE - POR FAVOR,NÃO O EDITE MANUALMENTE!

ATUALIZADO EM 21/05/2021 ÁS 18:35:38
*/

//INCREMENTE SEMPRE QUE HOUVER UM NOVO LANÇAMENTO
#define FIRMWARE_STORAGE_REVISION 10 //1.0

//NÚMERO DE BYTES DO ARMAZENAMENTO RESERVADOS PARA USO
#define TOTAL_SIZE_OF_STORAGE 2000

//ENDEREÇO PARA VERIFICAR SE É O PRIMEIRO UPLOAD DA VERSÃO DO FIRMWARE
#define FIRMWARE_MAGIC_ADDRESS 1500

//ENDEREÇOS PARA O CLI
#define KP_ACC_AHRS_ADDR 2
#define KI_ACC_AHRS_ADDR 3
#define KP_MAG_AHRS_ADDR 4
#define KI_MAG_AHRS_ADDR 5
#define AL_AHRS_BA_ADDR 6
#define AL_IMU_BA_ADDR 8
#define AL_IMU_GPS_VEL_ADDR 9
#define AL_TRIGGER_MOTOR_DELAY_ADDR 10
#define AL_ELEVATOR_ADDR 12
#define AL_SPINUP_ADDR 13
#define AL_SPINUP_TIME_ADDR 15
#define AL_MAX_THROTTLE_ADDR 17
#define AL_EXIT_ADDR 19
#define AL_ALTITUDE_ADDR 21
#define BATT_VOLTAGE_FACTOR_ADDR 25
#define BATT_AMPS_VOLT_ADDR 29
#define BATT_AMPS_OFFSET_ADDR 33
#define CC_BANKANGLE_ADDR 37
#define CC_TIME_ADDR 38
#define GIMBAL_MIN_ADDR 39
#define GIMBAL_MAX_ADDR 41
#define LAND_CHECKACC_ADDR 43
#define THROTTLE_FACTOR_ADDR 44
#define AUTODISARM_ADDR 48
#define AUTODISARM_THR_MIN_ADDR 50
#define AUTODISARM_YPR_MIN_ADDR 52
#define AUTODISARM_YPR_MAX_ADDR 54
#define WHEELS_ADDR 56
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
#define AS_AUTO_CAL_SCALE_ADDR 73

//ENDEREÇOS PARA AS CALIBRAÇÕES DOS SENSORES
#define ACC_ROLL_OFFSET_ADDR 481
#define ACC_PITCH_OFFSET_ADDR 483
#define ACC_YAW_OFFSET_ADDR 485
#define ACC_ROLL_SCALE_ADDR 487
#define ACC_PITCH_SCALE_ADDR 489
#define ACC_YAW_SCALE_ADDR 491
#define MAG_PITCH_OFFSET_ADDR 493
#define MAG_ROLL_OFFSET_ADDR 495
#define MAG_YAW_OFFSET_ADDR 497
#define MAG_ROLL_GAIN_ADDR 499
#define MAG_PITCH_GAIN_ADDR 501
#define MAG_YAW_GAIN_ADDR 503

//ENDEREÇOS PARA AS CONFIGURAÇÕES
#define SIMPLE_ADDR 541
#define ALT_HOLD_ADDR 542
#define GPS_HOLD_ADDR 543
#define RTH_ADDR 544
#define STABLIZE_ADDR 545
#define ATTACK_ADDR 546
#define PARACHUTE_ADDR 547
#define AUTOFLIP_ADDR 548
#define GIMBAL_ADDR 549
#define FRAME_TYPE_ADDR 550
#define FF_OR_CD_ROLL_ADDR 551
#define AUTOMISSION_ADDR 552
#define AUTOLAND_ADDR 553
#define ARMDISARM_ADDR 554
#define DISP_PASSIVES_ADDR 555
#define MAG_DECLINATION_ADDR 556
#define UART_NUMB_1_ADDR 560
#define UART_NUMB_2_ADDR 561
#define UART_NUMB_3_ADDR 562
#define COMPASS_ROTATION_ADDR 563
#define RTH_ALTITUDE_ADDR 564
#define THROTTLE_DZ_ADDR 565
#define YAW_DZ_ADDR 566
#define PITCH_DZ_ADDR 567
#define ROLL_DZ_ADDR 568
#define KP_ROLL_ADDR 569
#define KI_ROLL_ADDR 570
#define KD_ROLL_ADDR 571
#define KP_PITCH_ADDR 572
#define KI_PITCH_ADDR 573
#define KD_PITCH_ADDR 574
#define KP_YAW_ADDR 575
#define KI_YAW_ADDR 576
#define KD_YAW_ADDR 577
#define KP_VEL_Z_ADDR 578
#define KI_VEL_Z_ADDR 579
#define KD_VEL_Z_ADDR 580
#define KP_GPSPOS_ADDR 581
#define KI_GPSPOS_ADDR 582
#define KP_POS_Z_ADDR 583
#define KI_POS_Z_ADDR 584
#define KD_POS_Z_ADDR 585
#define KP_POS_RATE_ADDR 586
#define KI_POS_RATE_ADDR 587
#define KD_POS_RATE_ADDR 588
#define KP_NAV_RATE_ADDR 589
#define KI_NAV_RATE_ADDR 590
#define KD_NAV_RATE_ADDR 591
#define TPA_PERCENT_ADDR 592
#define BREAKPOINT_ADDR 593
#define HW_GYRO_LPF_ADDR 595
#define DERIVATIVE_LPF_ADDR 596
#define RC_LPF_ADDR 598
#define KALMAN_ADDR 600
#define BI_ACC_LPF_ADDR 601
#define BI_GYRO_LPF_ADDR 603
#define BI_ACC_NOTCH_ADDR 605
#define BI_GYRO_NOTCH_ADDR 607
#define MOT_COMP_STATE_ADDR 609
#define THR_ATTITUDE_MIN_ADDR 610
#define THR_ATTITUDE_MAX_ADDR 612
#define THROTTLE_MIDDLE_ADDR 614
#define THROTTLE_EXPO_ADDR 615
#define RC_RATE_ADDR 616
#define RC_EXPO_ADDR 617
#define ROLL_BANK_ADDR 618
#define PITCH_BANK_MIN_ADDR 619
#define YAW_RATE_ADDR 620
#define FF_OR_CD_PITCH_ADDR 621
#define FF_OR_CD_YAW_ADDR 622
#define SERVO1_RATE_ADDR 623
#define SERVO2_RATE_ADDR 625
#define SERVO3_RATE_ADDR 627
#define SERVO4_RATE_ADDR 629
#define FAILSAFE_VAL_ADDR 631
#define THROTTLE_MIN_ADDR 633
#define YAW_MIN_ADDR 635
#define PITCH_MIN_ADDR 637
#define ROLL_MIN_ADDR 639
#define THROTTLE_MAX_ADDR 641
#define YAW_MAX_ADDR 643
#define PITCH_MAX_ADDR 645
#define ROLL_MAX_ADDR 647
#define CH_REVERSE_ADDR 649
#define SERVO1_MIN_ADDR 650
#define SERVO2_MIN_ADDR 652
#define SERVO3_MIN_ADDR 654
#define SERVO4_MIN_ADDR 656
#define SERVO1_MID_ADDR 658
#define SERVO2_MID_ADDR 660
#define SERVO3_MID_ADDR 662
#define SERVO4_MID_ADDR 664
#define SERVO1_MAX_ADDR 666
#define SERVO2_MAX_ADDR 668
#define SERVO3_MAX_ADDR 670
#define SERVO4_MAX_ADDR 672
#define SERVOS_REVERSE_ADDR 674
#define SERVOS_LPF_ADDR 675
#define KP_AUTOLEVEL_ADDR 677
#define KI_AUTOLEVEL_ADDR 678
#define KP_HEADING_HOLD_ADDR 679
#define PITCH_BANK_MAX_ADDR 680
#define ATTACK_BANK_ADDR 681
#define GPS_BANK_ADDR 682
#define MAX_PITCH_LEVEL_ADDR 683
#define MAX_ROLL_LEVEL_ADDR 684
#define AIRSPEED_TYPE_ADDR 685
#define INTEGRAL_RELAX_LPF_ADDR 686
#define KCD_OR_FF_LPF_ADDR 688
#define PITCH_LEVEL_TRIM_ADDR 690

//CONFIGURAÇÕES E ENDEREÇOS PARA O MODO WAYPOINT
#define WAYPOINTS_MAXIMUM 10 //NÚMERO MAXIMO DE WAYPOINTS SUPORTADO
#define OTHERS_PARAMS_MAXIMUM 3 //ALTITUDE,TEMPO DO GPS-HOLD E O MODO DE VOO
#define INITIAL_ADDR_OF_COORDINATES 1000
#define FINAL_ADDR_OF_COORDINATES 1080
#define INITIAL_ADDR_OF_OTHERS_PARAMS 1084
#define FINAL_ADDR_OF_OTHERS_PARAMS 1495
