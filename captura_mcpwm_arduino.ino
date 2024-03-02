
/*

Ejemplo de como capturar el tiempo entre flancos de subida de 2 señales
usando el periferico MCPWM del ESP32. sacado del ejemplo de las misma librerias

#define GPIO_CAP0_IN   35  //Pin de entrada 35
#define GPIO_CAP1_IN   32   //Pin de enrada 32

https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/peripherals/mcpwm.html#api-reference
https://github.com/espressif/esp-idf/blob/v4.2/examples/peripherals/mcpwm/mcpwm_bldc_control/main/mcpwm_bldc_control_hall_sensor_example.c#L1

en cada flanco de subida se captura el temporizador y se envia a la cola

disp_captured_signal(void *arg): esta tarea lee de la cola  las capturas que ha echo la rutinade interrupcio
y dsi fue CAP0 o CAP1 en el ejemplo original existen 3 
luego calcula el periodo; guardando el ultimo valor leido que sera usado enel siguiente ciclo
luego imprime x la consola en microsegundos
115200



Arduino
Version: 2.3.2
Date: 2024-02-20T09:53:59.281Z
CLI Version: 0.35.3

*/

//================================================

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"


#define CAP_SIG_NUM 1   //two or 3 capture signals from GPIO_CAPx PINs
#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit

#define GPIO_CAP0_IN   35  //Set GPIO 25 as  CAP0
#define GPIO_CAP1_IN   32   //Set GPIO 26 as  CAP1


typedef struct {
    uint32_t capture_signal;
    mcpwm_capture_signal_t sel_cap_signal;
} capture;


static uint32_t hall_sensor_value = 0;
static uint32_t hall_sensor_previous = 0;

xQueueHandle cap_queue;

static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};

//=======================================================================================================
/**
 * @brief When interrupt occurs, we receive the counter value and display the time between two rising edge
 */
static void disp_captured_signal(void *arg)
{
    uint32_t *current_cap_value = (uint32_t *)malloc(sizeof(CAP_SIG_NUM));
    uint32_t *previous_cap_value = (uint32_t *)malloc(sizeof(CAP_SIG_NUM));
    uint32_t *dummy = (uint32_t *)malloc(sizeof(CAP_SIG_NUM));
    capture evt;
    while (1) {
        xQueueReceive(cap_queue, &evt, portMAX_DELAY);
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP0) {
            current_cap_value[0] = evt.capture_signal - previous_cap_value[0];
            previous_cap_value[0] = evt.capture_signal;
            current_cap_value[0] = (current_cap_value[0] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
            dummy[0] = (previous_cap_value[0] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
            printf("CAP0 : %d us\n", current_cap_value[0]);
            //printf("dummy : %d us\n", dummy[0]);
        }
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP1) {
            current_cap_value[1] = evt.capture_signal - previous_cap_value[1];
            previous_cap_value[1] = evt.capture_signal;
            current_cap_value[1] = (current_cap_value[1] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
            printf("CAP1 : %d us\n", current_cap_value[1]);
        }  
    }
   
    //vTaskDelay(pdMS_TO_TICKS(500)); 
}



//===========================================================================================
//Rutinade interrupcio

static void IRAM_ATTR isr_handler(void *arg)
{
    uint32_t mcpwm_intr_status;
    capture evt;
    mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status
    if (mcpwm_intr_status & CAP0_INT_EN) { //Check for interrupt on rising edge on CAP0 signal
        evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP0;
        xQueueSendFromISR(cap_queue, &evt, NULL);
    }
     if (mcpwm_intr_status & CAP1_INT_EN) { //Check for interrupt on rising edge on CAP1 signal
        evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP1); //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP1;
        xQueueSendFromISR(cap_queue, &evt, NULL);
    } 
    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;
}


//===================================================================================
void setup(void)
{

    printf("initializing mcpwm bldc control gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_IN); 
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, GPIO_CAP1_IN);
    //gpio_pulldown_en(GPIO_CAP0_IN);    //Enable pull down on CAP0   signal
    //gpio_pulldown_en(GPIO_CAP1_IN);    //Enable pull down on CAP1   signal
   
    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm bldc control...\n");
    mcpwm_config_t pwm_config;
   
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
  
    //3. Capture configuration
    //configure CAP0, CAP1 and CAP2 signal to start capture counter on rising edge
    //In general practice you can connect Capture  to external signal, measure time between rising edge or falling edge and take action accordingly
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, pulse num = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, pulse num = 0 i.e. 800,000,000 counts is equal to one second

    //enable interrupt, so each this a rising edge occurs interrupt is triggered
    MCPWM[MCPWM_UNIT_0]->int_ena.val = (CAP0_INT_EN | CAP1_INT_EN );  //Enable interrupt on  CAP0, CAP1 
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
    
    cap_queue = xQueueCreate(1, sizeof(capture)); 

    xTaskCreate(disp_captured_signal, "mcpwm_config", 4096, NULL, 2, NULL);  
    
}

void loop(){}
