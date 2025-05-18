#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "pico/bootrom.h"
#include <math.h>

//configuração do i2c para o display
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

//pinos do adc para o joystick
#define ADC_JOYSTICK_X 26
#define ADC_JOYSTICK_Y 27
#define DEADZONE 100

//pinos dos led RGB
#define LED_RED 13
#define LED_BLUE 12
#define LED_GREEN  11

#define buttonA 5
#define buttonB 6

#define BUZZER_PIN 10   //pino do buzzer

uint64_t last_time = 0;

//definção da estrutura que armazena os dados do joystick
typedef struct
{
    uint16_t x_position; //nivel da agua
    uint16_t x_center;
    float x_percentual;

    uint16_t y_position; //volume de chuva
    uint16_t y_center;
    float y_percentual;

} joystick_data_t;

QueueHandle_t xQueueJoystickData;

void gpio_irq_handler(uint gpio, uint32_t events)
{
    uint64_t current_time = to_ms_since_boot(get_absolute_time()); //armazena o registrado no momento da interrupção
    
    if(current_time - last_time > 200)  //checa se passou pelo menos 200 ms entre duas ativações do botão
    {
        if(gpio == buttonB) 
        {
            // npClear();  //limpa a matriz de leds
            // ssd1306_fill(&ssd, !color); // Limpa o display
            // ssd1306_send_data(&ssd);  // Atualiza o display
            reset_usb_boot(0, 0);   //coloca em modo bootloader
        }
        last_time = current_time; //atualiza as variáveis de tempo
    }
}

bool deadZoneCheck(uint16_t joy_position, uint16_t joy_center)
{
    if(joy_position > (joy_center - DEADZONE) && joy_position < (joy_center + DEADZONE))
    {
        return true; //retorna true se a leitura estiver dentro da zona morta
    }

    return false; //retorna false se a leitura estiver fora da zona morta
}

void vJoystickTask(void *params)
{
    adc_init();
    adc_gpio_init(ADC_JOYSTICK_Y);
    adc_gpio_init(ADC_JOYSTICK_X);

    joystick_data_t joydata;

    //bloco com leitura inicial do adc para definir onde é o centro do joystick
    sleep_ms(100); 

    adc_select_input(0);
    joydata.y_center = adc_read();
   
    adc_select_input(1);
    joydata.x_center = adc_read();

    printf("\nFirst Values x_center, y_center: %d, %d", joydata.x_center, joydata.y_position);

    sleep_ms(100);
    //fim da leitura inicial


    while (true)
    {
        adc_select_input(0); // GPIO 26 = ADC0
        joydata.y_position = adc_read();

        adc_select_input(1); // GPIO 27 = ADC1
        joydata.x_position = adc_read();

        //verifica se o eixo X está dentro da zona morta
        if(deadZoneCheck(joydata.x_position, joydata.x_center))
        {
            joydata.x_position = joydata.x_center;
        }

        //verifica se o eixo Y está dentro da zona morta
        if(deadZoneCheck(joydata.y_position, joydata.y_center))
        {
            joydata.y_position = joydata.y_center;
        }

       // float percentual = (fabs(adc_read - 2048) / 2047.0) * 100.0;


        joydata.x_percentual = (fabs((float)joydata.x_position - joydata.x_center)/joydata.x_center) * 100;    //calcula o percentual de X (nível da agua)

        joydata.y_percentual = (fabs((float)joydata.y_position - joydata.y_center)/joydata.y_center) * 100;  //calcula o percentual de  Y(volume de chuva)

        printf("\nADC_X: %d     X_CENTER: %d     X_PERCENTUAL: %f", joydata.x_position, joydata.x_center, joydata.x_percentual);
        printf("\nADC_Y: %d     Y_CENTER: %d     Y_PERCENTUAL: %f", joydata.y_position, joydata.y_center, joydata.y_percentual);

        xQueueSend(xQueueJoystickData, &joydata, 0); // Envia o valor do joystick para a fila
        vTaskDelay(pdMS_TO_TICKS(100));              // 10 Hz de leitura
    }
}

void vLedTask(void *params)
{
    gpio_init(LED_RED);
    gpio_init(LED_GREEN);

    gpio_set_dir(LED_RED, GPIO_OUT);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    

    joystick_data_t joydata;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
    
            //led vermelho se o nível da água for maior que 70% ou volume de chuva maior que 80%
            if(joydata.x_percentual > 70.0 || joydata.y_percentual > 80.0)
            {
                gpio_put(LED_RED, true);
                gpio_put(LED_GREEN, false);
                

            }//led amarelo se o nível da água for maior que 40% ou volume de chuva maior que 50%
            else if(joydata.x_percentual > 40.0 || joydata.y_percentual > 50.0) 
            {
                gpio_put(LED_RED, true);
                gpio_put(LED_GREEN, true);

            }else{
                //Desliga os led se o nível da água e volume de chuva estiverem normais
                gpio_put(LED_RED, false);
                gpio_put(LED_GREEN, false);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}




int main()
{
    stdio_init_all();

    gpio_init(buttonB);
    gpio_set_dir(buttonB, GPIO_IN);
    gpio_pull_up(buttonB);

    gpio_set_irq_enabled_with_callback(buttonB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    //cria a fila para os valores do joystick
    xQueueJoystickData = xQueueCreate(5, sizeof(joystick_data_t));

    //cria as tasks
    xTaskCreate(vJoystickTask, "Joystick Task", 256, NULL, 1, NULL);
    xTaskCreate(vLedTask, "led RGB Task", 256, NULL, 1, NULL);

    vTaskStartScheduler();
    panic_unsupported();


}
