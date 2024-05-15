#include <stdio.h>
#include <webots/supervisor.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <math.h>

#define TIME_STEP 256
#define QtddSensoresProx 8
#define QtddLeds 10

int main(int argc, char **argv) {
    int i = 0;
    char texto[256];
    double LeituraSensorProx[QtddSensoresProx];
    double AceleradorDireito = 1.0, AceleradorEsquerdo = 1.0;
    for (i = 0; i < 256; i++) texto[i] = '0';

    wb_robot_init();

    WbDeviceTag MotorEsquerdo, MotorDireito;

    MotorEsquerdo = wb_robot_get_device("left wheel motor");
    MotorDireito = wb_robot_get_device("right wheel motor");

    wb_motor_set_position(MotorEsquerdo, INFINITY);
    wb_motor_set_position(MotorDireito, INFINITY);

    wb_motor_set_velocity(MotorEsquerdo, 0);
    wb_motor_set_velocity(MotorDireito, 0);

    WbDeviceTag SensorProx[QtddSensoresProx];

    for (i = 0; i < QtddSensoresProx; i++) {
        char sensorName[5];
        sprintf(sensorName, "ps%d", i);
        SensorProx[i] = wb_robot_get_device(sensorName);
        wb_distance_sensor_enable(SensorProx[i], TIME_STEP);
    }

    WbDeviceTag Leds[QtddLeds];
    for (i = 0; i < QtddLeds; i++) {
        char ledName[5];
        sprintf(ledName, "led%d", i);
        Leds[i] = wb_robot_get_device(ledName);
        wb_led_set(Leds[i], 0); // Inicialmente, desligar todos os LEDs
    }
    
    WbNodeRef constCaixa = wb_supervisor_node_get_from_def("wooden_box");
    double vector[3] = {-0.0132922, 0.340513, 0.0499922};
    
    while (wb_robot_step(TIME_STEP) != -1) {
        for (i = 0; i < 256; i++) texto[i] = 0;

        for (i = 0; i < QtddSensoresProx; i++) {
            LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]) - 60;
            sprintf(texto, "%s|%d: %5.2f  ", texto, i, LeituraSensorProx[i]);
        }

        printf("%s\n", texto);

        const double *posicao_caixa_atual = wb_supervisor_node_get_position(constCaixa);

        printf("Posição da caixa atual: x=%.2f, y=%.2f, z=%.2f\n", 
               posicao_caixa_atual[0], posicao_caixa_atual[1], posicao_caixa_atual[2]);

        // Verifica se a posição da caixa mudou mais que 0.05 em qualquer direção (ajustado para sensibilidade)
        if (fabs(vector[0] - posicao_caixa_atual[0]) > 0.01 ||
            fabs(vector[1] - posicao_caixa_atual[1]) > 0.01 ||
            fabs(vector[2] - posicao_caixa_atual[2]) > 0.01) {
            printf("A caixa se moveu.\n");
            AceleradorDireito = 0;
            AceleradorEsquerdo = 0;
            wb_motor_set_velocity(MotorEsquerdo, 0);
            wb_motor_set_velocity(MotorDireito, 0);

            // Acender todos os LEDs para indicar sucesso
            for (i = 0; i < QtddLeds; i++) {
                wb_led_set(Leds[i], 1);
            }
            break;
        }

        if (LeituraSensorProx[0] > 10) {
            AceleradorDireito = -1;
            AceleradorEsquerdo = 1;
        } else {
            AceleradorDireito = 1.0;
            AceleradorEsquerdo = 1.0;
        }

        wb_motor_set_velocity(MotorEsquerdo, 6.28 * AceleradorEsquerdo);
        wb_motor_set_velocity(MotorDireito, 6.28 * AceleradorDireito);
    }

    wb_robot_cleanup();

    return 0;
}