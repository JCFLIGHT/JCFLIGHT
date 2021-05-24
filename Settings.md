# Variáveis do CLI

| Nome | Valor Padrão | Min | Max | Descrição |
| ------------- | ------------- | --- | --- | ----------- |
| AutoDisarm_Throttle_Min | 1100 | 800 | 1500 | Valor maximo do Throttle tolerado para iniciar a contagem do Auto-Desarmamento [uS] |
| AutoDisarm_Time | 5 | 0 | 255 | Estouro de tempo para desarmar a controladora em nivel baixo de Throttle [Segundos] |
| AutoLaunch_AHRS_BankAngle | 25 | 0 | 255 | Ângulo no AHRS para considerar que o Auto-Launch deve iniciar [Graus] |
| AutoLaunch_Altitude | 0 | 0 | 255 | Cancela o Auto-Launch após atingir essa altitude (O tempo acima será ignorado) [Metros] |
| AutoLaunch_Elevator | 18 | 0 | 100 | Inclinação no Pitch (Elevator) ao fazer o Auto-Launch [Graus] |
| AutoLaunch_Exit | 5000 | 0 | 30000 | Cancela o Auto-Launch após o estouro desse tempo [MillisSegundos] |
| AutoLaunch_IMU_BankAngle | -450 | -1000 | 1000 | Ângulo na IMU para considerar que o Auto-Launch deve iniciar [Graus*10] |
| AutoLaunch_MaxThrottle | 1700 | 1000 | 2000 | Valor do Throttle aplicado ao motor durante o Auto-Launch [uS] |
| AutoLaunch_SpinUp | 100 | 0 | 2000 | Valor de incrimentação no Throttle para Aeros com rodas [uS] |
| AutoLaunch_SpinUp_Time | 300 | 0 | 5000 | Tempo de incrimentação no Throttle para Aeros com rodas [MillisSegundos] |
| AutoLaunch_Trigger_Motor_Delay | 1500 | 0 | 10000 | Tempo para iniciar o motor após o status de lançado [MillisSegundos] |
| AutoLaunch_Velocity_Thresh | 3 | 0 | 20 | Velocidade da IMU ou GPS para validar o Auto-Launch [Metros/Segundo] |
| Batt_Amps_OffSet | 0 | 0 | 1000 | Ajuste fino do valor da Corrente |
| Batt_Amps_Per_Volt | 62.0 | 0 | 1000 | Fator de calibração para converter a leitura ADC em Corrente |
| Batt_Voltage_Factor | 10.1 | 0 | 1000 | Fator de calibração para converter a leitura ADC em Tensão |
| CrashCheck_BankAngle | 3 | 0 | 20 | Valor da aceleração da IMU [Metros/Segundo^2] |
| CrashCheck_Timer | 2 | 0 | 255 | Estouro de tempo para validar o Crash [Segundos] |
| GimbalMaxValue | 2000 | 800 | 2200 | Valor maximo do pulso a ser aplicado no Gimbal [uS] |
| GimbalMinValue | 1000 | 800 | 2200 | Valor minimo do pulso a ser aplicado no Gimbal [uS] |
| ThrottleMixGain | 1.0 | 0 | 1 | Valor de ganho do Throttle para o mixer do PID |
| kI_Acc_AHRS | 50 | 0 | 255 | Ganho Integral para correção da estimativa de Attitude |
| kI_Mag_AHRS | 0 | 0 | 255 | Ganho Integral para correção da estimativa de direção do Yaw |
| kP_Acc_AHRS | 25 | 0 | 255 | Ganho Proporcional para correção da estimativa de Attitude |
| kP_Mag_AHRS | 10 | 0 | 255 | Ganho Proporcional para correção da estimativa de direção do Yaw |

> Esse arquivo é gerado automaticamente,não o edite manualmente!