/*
------------------------
| 3Cadeira - Protótipo |
------------------------
Programador: Melvin Gustavo Maradiaga Elvir

Neste código vamos implementar o algoritmo que vai realizar o controle mais formal da cadeira de rodas,
a depender do botão que for apertado.

A depender do botão que for apertado, realizamos o controle dos sinais de entrada da cadeira. Vou
utilizar a interrupção ext1 do ESP32, que me permite acordar o microcontrolador de vários pinos do RTC
ao mesmo tempo. 

O que preciso fazer é registrar certinho qual é o botão que está acionando minha interrupção e depois
passar esses dados ao micro após ele acordar. 
*/
#include <stdio.h>
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/rtc_io.h"

#define PINO_EIXO_X GPIO_NUM_18
#define PINO_EIXO_Y GPIO_NUM_19
#define NUM_TIMER_CHANNELS 2
static enum channels_pwm{
    CHANNEL_Y,
    CHANNEL_X
};

// Aqui crio um número NUM_TIMER_CHANNELS de structs contendo as importantes para ledc_channel.
static ledc_channel_config_t  ledc_channel[NUM_TIMER_CHANNELS] = {
    {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = PINO_EIXO_Y,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0,
    },
    {
        .channel = LEDC_CHANNEL_1,
        .duty = 0,
        .gpio_num = PINO_EIXO_Y,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0,
    }
};

// Aqui defino a variável que quero utilizar depois de sair do deep_sleep
void app_main() {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    int num_botao = 0;
    /*
    Aqui conferimos o que fez o ESP32 acordar do deep_sleep. Caso NÃO tenha sido
    a interrupção EXT1, voltamos a executar ele (Isto é pois o botão de reset é uma interrupção
    existente no vetor de interrupções no ESP32).
    Caso tenha sido a interrupção EXT1, executamos o código que queremos seja executado.
    */ 
    if (cause != ESP_SLEEP_WAKEUP_EXT1) {
        printf("Not EXT1 wakeup, ESP32 was resetted.\n");
        config_hw();
    } else {
        printf("EXT1 wakeup, I'll write the voltage values into the pins.\n");
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        /*
        Aqui vou conferir qual foi o pino que gerou a interrupção EXT1, mediante a função
         _builtin_ffsll(wakeup_pin_mask).
        */
        if (wakeup_pin_mask != 0) {
            num_botao = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", num_botao);
        } else {
            printf("Wake up from GPIO\n");
        }
        // Talvez faça sentido trocar posição deste aqui.
        write_joystick_direction(num_botao);
    }
    printf("Entering deep sleep.\n\n");
    esp_deep_sleep_start();
}

static void config_hw(){
    /*
    Nesta primeira parte configuro os pinos que irei utilizar para a escrita dos valores PWM
    nos pinos de saída que vão alimentar o circuito principal.
    Note-se que primeiro preciso configurar os pinos que irei utilizar, qual será o seu estado de funcionamento
    normal, para depois realizar a escrita do duty cycle correspondente.
    */
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_LED_PIN_SEL;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    /*
    [ Configurações do Timer e Canais do PWM ]

    Nesta seção do código realizo a configuração do timer e dos canais que utilizarei neste projeto
    para gerar os sinais PWM que vão sair dos meus dois pinos (18 e 19). Note-se que por estarmos usando
    o ESPIDF estamos precisando configurar os structs ou, como gosto de chamar, "pseudo-registradores" para
    definir certinho os parametros dos nossos periféricos.
    */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 1000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    // Aqui realizo a escrita das configurações dos canais de PWM que pretendo utilizar.
    for (int ch = 0; ch < NUM_TIMER_CHANNELS; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    /*
    Aqui deixo claro os pinos que pretendo utilizar dentro da minha aplicação para ativar o EXT1.
    A escolha dos pinos não foi arbritária. Precisamos usar pinos que sejam de RTC e além disso que sejam
    pinos capazes de me falar de volta que ELES foram os que executaram a interrupção.
    */
    gpio_num_t gpio_1 = GPIO_NUM_25;
    gpio_num_t gpio_2 = GPIO_NUM_26;
    gpio_num_t gpio_3 = GPIO_NUM_27;
    
    /*
    Aqui habilito os pinos do botão para serem detectados no modo deep_sleep.
    Note-se que como estamos lidando com pinos de RTC que tem VÁRIAS funcionalidades, preciso
    deixar claro quais coisas eu não quero estejam funcionando com eles.
    No caso, preciso desabilitar o pullup e pulldown, além de estabelecer que eles vão servir como
    pinos de entrada.
    */
    rtc_gpio_init(gpio_1);
    rtc_gpio_init(gpio_2);
    rtc_gpio_init(gpio_3);
    rtc_gpio_set_direction(gpio_1, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_set_direction(gpio_2, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_set_direction(gpio_3, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_dis(gpio_1);
    rtc_gpio_pullup_dis(gpio_2);
    rtc_gpio_pullup_dis(gpio_3);
    rtc_gpio_pulldown_en(gpio_1);
    rtc_gpio_pulldown_en(gpio_2);
    rtc_gpio_pulldown_en(gpio_3);
    rtc_gpio_hold_en(gpio_1);
    rtc_gpio_hold_en(gpio_2);
    rtc_gpio_hold_en(gpio_3);

    // Aqui desabilitamos uns pinos que parece vão aumentar o consumo energético do ESP32 após
    // ele ser ligado.
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    esp_deep_sleep_disable_rom_logging();

    /*
    Nesta seção configuro qual será o principal motivo para o ESP32 acordar do deepsleep. Para isso, 
    utilizo o esp_enable_ext1_wakeup( #1, #2 ) que toma dois parametros:
    #1 : O bitmask dos pinos que pretendo permitir consigam acordar o meu dispositivo.
         Aqui, pensemos nos 39 pinos GPIO do ESP32 como sendo 39bits ndistintos em um registrador.
         GPIO 0 corresponde ao bit menos signficativo enquanto GPIO 39 corresponde ao bit mais significativo.
         Assim: GPIO39-GPIO0 : 0x000000000
    #2 : Aqui defino qual estado o ESP32 deve registrar para fazer o sistema acordar do deepsleep. 
         No meu caso, quero que ele acorde quando a entrada em qualquer botão vire HIGH.
    */
    esp_err_t err = esp_sleep_enable_ext1_wakeup((1ULL<GPIO_NUM_25)|(1ULL<GPIO_NUM_26)|(1ULL<GPIO_NUM_27),ESP_EXT1_WAKEUP_ANY_HIGH);
    ESP_ERROR_CHECK( err );
}

/*
> write_joystick_direction <

Esta função vai escrever um certo valor num pino de saida a depender do valor que foi registrado como ter
ativado a minha interrupção ext1. A depender do pino que tenha feito o trabalho, vou escrever uma série
de valores nas saidas PWM do ESP32 aproveitando a biblioteca ledc nativa do ESP32.
Para fazer esse sinal ser uma tensão AC, vamos utilizar um circuito simples retificador na saida.
*/
static void write_joystick_direction(int num_botao){
    switch(num_botao){
        case 25:
            // Aqui configuro o duty cycle dos meus dois pinos a depender do seu canal.
            ledc_set_duty(ledc_channel[CHANNEL_X].speed_mode, ledc_channel[CHANNEL_X].channel, 2000);
            ledc_update_duty(ledc_channel[CHANNEL_X].speed_mode, ledc_channel[CHANNEL_X].channel);
            ledc_set_duty(ledc_channel[CHANNEL_Y].speed_mode, ledc_channel[CHANNEL_Y].channel, 4000);
            ledc_update_duty(ledc_channel[CHANNEL_Y].speed_mode, ledc_channel[CHANNEL_Y].channel);
            vTaskDelay(pdMS_TO_TICKS(500));
            break;
        case 26:
            ledc_set_duty(ledc_channel[CHANNEL_X].speed_mode, ledc_channel[CHANNEL_X].channel, 4000);
            ledc_update_duty(ledc_channel[CHANNEL_X].speed_mode, ledc_channel[CHANNEL_X].channel);
            ledc_set_duty(ledc_channel[CHANNEL_Y].speed_mode, ledc_channel[CHANNEL_Y].channel, 2000);
            ledc_update_duty(ledc_channel[CHANNEL_Y].speed_mode, ledc_channel[CHANNEL_Y].channel);
            vTaskDelay(pdMS_TO_TICKS(500));
            break;
        case 27:
            ledc_set_duty(ledc_channel[CHANNEL_X].speed_mode, ledc_channel[CHANNEL_X].channel, 10);
            ledc_update_duty(ledc_channel[CHANNEL_X].speed_mode, ledc_channel[CHANNEL_X].channel);
            ledc_set_duty(ledc_channel[CHANNEL_Y].speed_mode, ledc_channel[CHANNEL_Y].channel, 2000);
            ledc_update_duty(ledc_channel[CHANNEL_Y].speed_mode, ledc_channel[CHANNEL_Y].channel);
            vTaskDelay(pdMS_TO_TICKS(500));
            break;
        default:
            printf("O valor recebido foi distinto de 25, 26 e 27.");
            break;
    }
}

