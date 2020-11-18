/*
 * File:          Aula02_Controlador.c
 * Date: 04/11/2020
 * Description: Atividade IA
 * Author: Filipi Guimarães Silva
 * Modifications:
 */

#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/accelerometer.h>
#include <webots/led.h>

#define TIME_STEP 256
#define QtddSensoresProx 8
#define QtddLeds 10
#define QtdAceleracao 5


WbDeviceTag Leds[QtddLeds];
void setLeds(int qtdLeds, int valor);

int main(int argc, char** argv)
{
    int i = 0, countCaixasMoveis = 0;
    char texto[256];
    double LeituraSensorProx[QtddSensoresProx];
    double AceleradorDireito = 1.0, AceleradorEsquerdo = 1.0;
    bool detectouCaixaMovel = false;
    double aceleracao[QtdAceleracao];
    
    for (i = 0; i < QtdAceleracao; i++) aceleracao[i] = 0;

    for (i = 0; i < 256; i++) texto[i] = '0';

    wb_robot_init();

    WbDeviceTag MotorEsquerdo, MotorDireito, Acelerometro;

    MotorEsquerdo = wb_robot_get_device("left wheel motor");
    MotorDireito = wb_robot_get_device("right wheel motor");
    Acelerometro = wb_robot_get_device("accelerometer");

    wb_accelerometer_enable(Acelerometro, 10);

    const double* values = wb_accelerometer_get_values(Acelerometro);

    wb_motor_set_position(MotorEsquerdo, INFINITY);
    wb_motor_set_position(MotorDireito, INFINITY);

    wb_motor_set_velocity(MotorEsquerdo, 0);
    wb_motor_set_velocity(MotorDireito, 0);

    WbDeviceTag SensorProx[QtddSensoresProx];

    Leds[0] = wb_robot_get_device("led0");
    Leds[1] = wb_robot_get_device("led1");
    Leds[2] = wb_robot_get_device("led2");
    Leds[3] = wb_robot_get_device("led3");
    Leds[4] = wb_robot_get_device("led4");
    Leds[5] = wb_robot_get_device("led5");
    Leds[6] = wb_robot_get_device("led6");
    Leds[7] = wb_robot_get_device("led7");
    Leds[8] = wb_robot_get_device("led8");
    Leds[9] = wb_robot_get_device("led9");

    setLeds(QtddLeds, 0);

    SensorProx[0] = wb_robot_get_device("ps0");
    SensorProx[1] = wb_robot_get_device("ps1");
    SensorProx[2] = wb_robot_get_device("ps2");
    SensorProx[3] = wb_robot_get_device("ps3");
    SensorProx[4] = wb_robot_get_device("ps4");
    SensorProx[5] = wb_robot_get_device("ps5");
    SensorProx[6] = wb_robot_get_device("ps6");
    SensorProx[7] = wb_robot_get_device("ps7");

    wb_distance_sensor_enable(SensorProx[0], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[1], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[2], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[3], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[4], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[5], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[6], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[7], TIME_STEP);
    
    int iteracoes = 0;
    while (wb_robot_step(TIME_STEP) != -1)
    {

        for (i = 0; i < 256; i++) texto[i] = 0;

        for (i = 0; i < QtddSensoresProx; i++)
        {
            LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]);
        }

        aceleracao[iteracoes % QtdAceleracao] = values[0] - values[1];

        double aceleracaoMedia = 0;
        for (i = 0; i < QtdAceleracao; i++) aceleracaoMedia += aceleracao[i];
        aceleracaoMedia = aceleracaoMedia / QtdAceleracao;
        
        if (aceleracaoMedia > 0.04 || aceleracaoMedia < -0.04)
        {
            detectouCaixaMovel = false;

            if (LeituraSensorProx[0] > 500)
            {
                AceleradorDireito = 1;
                AceleradorEsquerdo = -0.1;
            }
            else if (LeituraSensorProx[1] > 400)
            {
                AceleradorDireito = 1;
                AceleradorEsquerdo = -0.2;
            }
            else if (LeituraSensorProx[2] > 400)
            {
                AceleradorDireito = 1;
                AceleradorEsquerdo = -0.3;
            }
            else if (LeituraSensorProx[7] > 400)
            {
                AceleradorDireito = -0.1;
                AceleradorEsquerdo = 1;
            }
            else if (LeituraSensorProx[6] > 400)
            {
                AceleradorDireito = -0.2;
                AceleradorEsquerdo = 1;
            }
            else if (LeituraSensorProx[5] > 400)
            {
                AceleradorDireito = -0.3;
                AceleradorEsquerdo = 1;
            }
            else
            {
                AceleradorDireito = 1;
                AceleradorEsquerdo = 1;
            }
        }
        else
        {
            AceleradorDireito = 1;
            AceleradorEsquerdo = 1;
            if (!detectouCaixaMovel && (LeituraSensorProx[0] > 1000 || LeituraSensorProx[1] > 1000 || LeituraSensorProx[6] > 1000 || LeituraSensorProx[7] > 1000))
            {
                for(i = 0; i < 3; i++)
                {
                    setLeds(10, 1);
                    wb_robot_step(100);
                    setLeds(10, 0);
                    wb_robot_step(100);
                }
                detectouCaixaMovel = true;
                printf("Quantidade de caixas móveis detectadas: %d;\n", ++countCaixasMoveis);
                
                for(i = 0; i < countCaixasMoveis; i++)
                {
                    setLeds(countCaixasMoveis, 1);
                }
            }
        }

        wb_motor_set_velocity(MotorEsquerdo, 4.28 * AceleradorEsquerdo);
        wb_motor_set_velocity(MotorDireito, 4.28 * AceleradorDireito);

        iteracoes++;
    };

    wb_robot_cleanup();

    return 0;
}


void setLeds(int qtdLeds, int valor)
{
    for(int i = 0; i < qtdLeds; i++){
       wb_led_set(Leds[i], valor);
    }
}
