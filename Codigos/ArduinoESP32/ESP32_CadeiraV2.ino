/*
-------------------------------------
| 3Cadeira (Arduino) - Protótipo V2 |
-------------------------------------
Programador: Melvin Gustavo Maradiaga Elvir
V0 e V1 - 23/10/2023
V2 - 24/10/2023 : Nesta versão acrescento as funcionalidades do ULP e o apertar do botão.

Neste código vamos implementar o algoritmo que vai realizar o controle mais formal da cadeira de rodas,
a depender do botão que for apertado, só que utilizando a Arduino framework.
A ideia é implementar o código de uma forma fácil de entender que permita, depois, a importação para o 
ESPIDF caso o objetivo for comercializar em menos escala o código desenvolvido. 

Neste programa iremos a utilizar as seguintes capacidades do ESP32:
- Modo de DEEPSLEEP com o ULP (Ultra Low Power Coprocessor) ativo.
- Pinos de RTC (Pinos do Real Time Clock interno do ESP32: Pinos 25, 26 e 27)
- Pinos de saida PWM para gerar uma tensão de 0V - 3,3V (Pinos 18 e 19)
- Interrupção EXT1 do ESP32
- Timer interno do ESP32 (Timer0)

A ideia de funcionamento deste programa é a seguinte: 
Estou querendo usar o microcontrolador como uma interface de baixo consumo energético que detecte
qual botão foi apertado e encaminhe um sinal DC aos pinos do potenciometro (joystick) no controle
da cadeira de rodas. Para isso, vamos estar monitorando o estado de três botões externos ao ESP32

Fluxograma:

-- ESP32 Acordado --
[Inicio]--->[Diretivos do preprocessador.]
            |
            |--> [Entro na função setup()]
            
[setup()]--->[]
             |
             |--> [Entro no LOOP]

[main()]--->[]

-- ESP32 em Deep Sleep --
[ULP]--->
*/

/*
-------------------
| Pre-processador |
-------------------
Neste bloco de código realizo a inclusão de bibliotecas e a definição de constantes.
No caso das bibliotecas, elas são bibliotecas nativas do ESP32, pelo qual vai se perceber sua
sintaxe é a mesma tanto na framework do Arduino como em ESPIDF.
Caso eu precise modificar alguma funcionalidade do código (como, por exemplo, quais pinos
irão fazer o ESP32 sair do DEEPSLEEP), só precisamos alterar o valor de algumas constantes
do código.

-- Bibliotecas --
(i) esp32/ulp.h: Esta biblioteca permite a configuração do ULP do ESP32.
(ii) driver/rtc_io.h: Esta biblioteca permite a configuração dos pinos do RTC.

-- Constantes -- 
(i) EXT1BITMASK:
(ii) PINO_EIXO_X:
(iii) PINO_EIXO_Y:
(iv) PRESCALER_TIMERS:
(v) LEDC_TIMER_8_BIT:

-- Constantes de Teste --
(i) TESTE_SAIRDOSLEEP:
(ii) TESTE_BOTAO:
*/
#include "esp32/ulp.h"
#include "driver/rtc_io.h"

#define EXT1BITMASK (1ULL<<GPIO_NUM_25)|(1ULL<<GPIO_NUM_26)|(1ULL<<GPIO_NUM_27)
#define PINO_EIXO_X GPIO_NUM_13
#define PINO_EIXO_Y GPIO_NUM_14
#define PRESCALER_TIMERS 80
#define LEDC_TIMER_8_BIT 8
#define TEMPO_DEBOUNCE 10 //Em ms
//#define TESTE_SAIRDOSLEEP
//#define TESTE_BOTAO
#define TESTE_ULP


/*
---------------------------
| Declaração de Variáveis |
---------------------------
Neste segmento do código declaro algumas variáveis que irei utilizar no meu programa, que preciso
sejam globais para fins de uso no escopo mais geral do programa.

Sob a hipotese que o ESP32 com Arduino framework já implementa o FreeRTOS interno do 
microcontrolador, irei declarar as minhas variáveis globais com static. Aquelas variáveis
que eu manipulo dentro de uma SRI eu declaro como volatile, assim o compilador (e suas otimizações)
não briga muito com elas.

Explicando cada variável declarada:
(i) num_botao: Esta variável vai armazenar o número de pino que fez o meu sistema acordar do DEEPSLEEP,
               por conta da interrupção EXT1.
(ii) channels_pwm: Aqui defino um enum que simplesmente organiza de uma forma mais bonita os canais que 
                   usarei para cada saída PWM. Note-se que estes correspondem ao funcionamento do PWM
                   quando o ESP32 está acordado.
(iii) temporizadores: Aqui defino um enum que organiza de forma mais bonita os meus temporizadores,
                      caso queira no futuro implementar mais de um.
(iv) flag_deepSleep: Este é um tipo de variável especial que será armazenado na memória RTC do ESP32.
                      Assim, ao entrar em DEEPSLEEP o meu ESP32 ainda pode modificar esta variável, e
                      esse valor ainda existe após acordar o ESP32.
(v) num_s: Esta variável será o contador que vou estar atualizando dentro do temporizador 0.
             Este contador precisa ser de 8bits para contar até os 3s desejados por mim.
             Como ela é manipulada dentro de uma SRI, defino ela como sendo volatil.
(vi) *timer0_cfg: Esta seção cria um struct hw_timer_t contendo informações de configuração
                  do temporizador. Ele é declarado como NULL pois na prática, não aponta a nenhum
                  espaço da SRAM do ESP32.
(vii) status_gpio: São variáveis que irei utilizar para acompanahar se os meus botões estão apertados
                   ou não.
*/
static volatile uint16_t num_botao = 0;
enum channels_pwm{
  CHANNEL_Y,
  CHANNEL_X
};
enum temporizadores{
  TEMPORIZADOR_0,
  TEMPORIZADOR_1,
  TEMPORIZADOR_2,
  TEMPORIZADOR_3
};
RTC_DATA_ATTR int flag_deepSleep = 0;
volatile uint16_t num_t0s = 0;
volatile uint16_t num_t1s = 0;
volatile uint16_t num_t2s = 0;
volatile uint16_t num_t3s = 0;
hw_timer_t *timer0_cfg = NULL;
hw_timer_t *timer1_cfg = NULL;
hw_timer_t *timer2_cfg = NULL;
hw_timer_t *timer3_cfg = NULL;
volatile uint8_t status_gpio25 = 0;
volatile uint8_t status_gpio26 = 0;
volatile uint8_t status_gpio27 = 0;
volatile uint8_t num_accesos = 0;

/*
-------------------------------------
| SRIs (Sub-rotinas de Interrupção) |
-------------------------------------
Nesta seção declaro as SRIs que irei utilizar no meu programa. Note-se que elas
foram intencionalmente implementadas da forma mais simples possível, para não ter muito
problema com o compilador durante a compilação.

As minhas SRIs são as seguintes:
(i) Timer0_ISR(): Sempre que o ALARM do meu timer0 foi ativado, aumento o valor armazenado
                  na variável num_s.
(ii) GPIOX_ISR(): Nesta SRI seto a flag correspondente a GPIO_X para saber se ele está apertado
                  ou não está apertado. A depender do estado, escrevo o valor do pino que ativou
                  a interrupção em num_botao.
*/

void IRAM_ATTR Timer0_ISR(){
  num_t0s++;
}

void IRAM_ATTR Timer1_ISR(){
  if(digitalRead(GPIO_NUM_25) == 1){
    num_botao = GPIO_NUM_25;
    status_gpio25 = 1;
    status_gpio26 = 0;
    status_gpio27 = 0;
  } else if(digitalRead(GPIO_NUM_26) == 1){
    num_botao = GPIO_NUM_26;
    status_gpio25 = 0;
    status_gpio26 = 1;
    status_gpio27 = 0;
  } else if(digitalRead(GPIO_NUM_27) == 1){
    num_botao = GPIO_NUM_27;
    status_gpio25 = 0;
    status_gpio26 = 0;
    status_gpio27 = 1;
  } else {
    status_gpio25 = 0;
    status_gpio26 = 0;
    status_gpio27 = 0;
    num_botao = 0;
  }
}

/*
--------------------------
| Função de Configuração |
--------------------------
Nesta função configuro os periféricos que irei utilizar neste programa. Dentro do Arduino IDE,
a função setup() contem códigos que serão executados uma única vez durante o funcionamento
do ESP32.

Assim, realizo as seguintes configurações:
(i) Ativar e configurar as UARTs do ESP32 para comunicação serial.
    * Isso aqui é feito com a função Serial.begin(115200). Note-se que defini meu baudrate
      como sendo 115200 bps.
(ii) Ativar e configurar os Timers que irei utilizar para o sinal PWM que vai alimentar o
     pino do potenciometro no controlador.
     * Assim, configuro primeiro os meus temporizadores com a função:
       ledcSetup(CANAL_TEMPORIZADOR, FREQUÊNCIA, RESOLUÇÃO)
     * Depois associo os pinos do ESP32 a um canal do temporizador com a função:
       ledcAttachPin(PINO, CANAL_TEMPORIZADOR)
(iii) Configurar a interrupção EXT1 como capaz de acordar o ESP32 se algum dos pinos associados
      estiver HIGH.
(iv) Configurar o Timer2 que usarei para minha contagem de ciclos para voltar ao modo deepsleep do ESP32.
     O ESP32 já utiliza um relógio de 80MHz (APB_CLK) por default com os seus temporizadores.
     Para utilizar uma frequência menor no nosso temporizador, utilizamos uma ferramenta (um circuito) interna
     dele conhecido como o prescaler.
     O prescaler essencialmente limita a nossa frequência de CPU, fazendo um AND com ela para gerar nossa
     frequência útil conforme mostro abaixo:

     -----------
     | Relógio |------>|\
     -----------       | \      -------------------
                       |  \ --> | Frequência útil |
                       |  /     -------------------
     -------------     | /
     | Prescaler | --->|/
     -------------

     A função que usamos para programar o nosso temporizador no ESP é a seguinte:
     hw_timer_t * timerBegin(uint8_t timerNumber, uint16_t prescaler, bool countUpOrDown);
     onde:
     * timerNumber: Especifica qual é o temporizador do ESP32 (por conta dele ter vários)
                    que irei utilizar desta vez.
     * prescaler: O que expliquei anteriormente.
     * countUpOrDown: Em que direção será feita a contagem interna dentro do timer.
                      (Exemplo: Se vai ir de 0 a 1000 ou de 1000 a 0)

     A função que usamos para configurar a interrupção associada ao timer é a seguinte:
     void timerAttachInterrupt(hw_timer_t *timer, void (*fn)(void), bool interruptMode)

     Assim escolho utilizar os seguintes parâmetros para o meu temporizador:
     ----------------------- 
     * Parâmetros do Timer *
     -----------------------
     Frequência do CLK : 80 MHz = 80.000.000 Ciclos por segundo
     Prescaler escolhido :  80
     Frequência do CLK do Timer:
*/

void setup() {
  // Aqui defino os parâmetros dos meus timers e os canais associados.
  ledcSetup(CHANNEL_X, 4000, LEDC_TIMER_8_BIT);
  ledcSetup(CHANNEL_Y, 4000, LEDC_TIMER_8_BIT);
  ledcAttachPin(PINO_EIXO_X, CHANNEL_X);
  ledcAttachPin(PINO_EIXO_Y, CHANNEL_Y);
  
  timer0_cfg = timerBegin(TEMPORIZADOR_0, PRESCALER_TIMERS, true);
  timerAttachInterrupt(timer0_cfg, &Timer0_ISR, true);
  timerAlarmWrite(timer0_cfg, 10000, true);
  timer1_cfg = timerBegin(TEMPORIZADOR_1, PRESCALER_TIMERS, true);
  timerAttachInterrupt(timer1_cfg, &Timer1_ISR, true);
  timerAlarmWrite(timer1_cfg, 15000, true);

  if(flag_deepSleep){
    timerAlarmEnable(timer0_cfg);
    timerAlarmEnable(timer1_cfg);
  }

  #if defined(TESTE_SAIRDOSLEEP) || defined(TESTE_BOTAO)
    // Coloco o delay neste segmento para permitir o Serial ser, de fato, ativado.
    Serial.begin(115200);
    delay(1000);
    // Aqui escrevo no monitor serial o motivo pelo qual meu ESP32 acordou.
  #endif
  obter_motivo_acordar();

  if(flag_deepSleep){
    pinMode(GPIO_NUM_25, INPUT);
    pinMode(GPIO_NUM_26, INPUT);
    pinMode(GPIO_NUM_27, INPUT);
  }
  // Aqui defino que a interrupção EXT1 vai acordar o meu ESP32 do sleep.
  esp_sleep_enable_ext1_wakeup(EXT1BITMASK ,ESP_EXT1_WAKEUP_ANY_HIGH);
}

/*
---------------
| Função Loop |
---------------
*/
void loop(){
  if(!flag_deepSleep){
    // Como o periferico utilizado para encaminhar as mensagens do serial demora vários ciclos de relógio
    // para encaminhar os caracteres, precisamos colocar esse delay para garantir que a mensagem seja encaminhada.
    // Neste caso, é ainda mais importante por conta do deep_sleep colocado logo depois.
    #ifdef TESTE_SAIRDOSLEEP
      Serial.println("Entrando em modo deep sleep");
      delay(1000);
    #endif
    flag_deepSleep = 1;
    #ifdef TESTE_ULP
      init_ulp();
    #endif
    esp_deep_sleep_start();
  } else {
    if(num_t0s > 300){ 
      flag_deepSleep = 0;
    }
    if((status_gpio25 == 1)||(status_gpio26 == 1)||(status_gpio27 == 1)){
      num_t0s = 0;
    }
    write_joystick_direction(num_botao);
  }
}

/*
-----------------------------------
| Função write_joystick_direction |
-----------------------------------
*/
void write_joystick_direction(int num_botao){
  switch(num_botao){
    case 0:
      ledcWrite(CHANNEL_X, 125);
      ledcWrite(CHANNEL_Y, 125);
      #ifdef TESTE_BOTAO
        Serial.println("Modo 0");
        Serial.print("num_t0s:");
        Serial.println(num_t0s);
        delay(1000);
      #endif
      break;  
    case 25:
      ledcWrite(CHANNEL_X, 255);
      ledcWrite(CHANNEL_Y, 125);
      #ifdef TESTE_BOTAO
        Serial.println("Modo 1");
        Serial.print("num_t1s:");
        Serial.println(num_t1s);
        Serial.print("num_t2s:");
        Serial.println(num_t2s);
        Serial.print("num_t3s");
        Serial.println(num_t3s);
        delay(1000);
      #endif
      break;
    case 26:
      ledcWrite(CHANNEL_X, 0);
      ledcWrite(CHANNEL_Y, 125);
      #ifdef TESTE_BOTAO
        Serial.println("Modo 2");
        delay(1000);
      #endif
      break;
    case 27:
      ledcWrite(CHANNEL_X, 125);
      ledcWrite(CHANNEL_Y, 255);
      #ifdef TESTE_BOTAO
        Serial.println("Modo 3");
        delay(1000);
      #endif
      break;
    default:
      #ifdef TESTE_BOTAO
        Serial.println("O valor recebido foi distinto de 25, 26 e 27.");
        delay(1000);
      #endif
      break;
    }
}

/*
---------------------------------
| Função obter_motivo_acordar() |
---------------------------------
*/
void obter_motivo_acordar(){
  // Aqui defino uma variável que vai acompanhar o motivo pelo qual sai do DEEPSLEEP. 
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason != ESP_SLEEP_WAKEUP_EXT1){
    #ifdef TESTE_SAIRDOSLEEP
      Serial.println("Wakeup caused by ESP32 normal reset.\n");
      delay(1000);
    #endif
  } else {
    uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
    #ifdef TESTE_SAIRDOSLEEP
      Serial.println("Wakeup caused by external signal using RTC_IO\n");
      delay(1000);
    #endif
    if(wakeup_pin_mask != 0){
      num_botao = __builtin_ffsll(wakeup_pin_mask) - 1;
      #ifdef TESTE_SAIRDOSLEEP
        Serial.print("Wakeup from GPIO");
        Serial.println(num_botao);
        delay(1000);
      #endif
    } 
  }
}

/*
---------------------
| Função init_ulp() |
---------------------
*/
#ifdef TESTE_ULP
void init_ulp(void){
  // Inicializo a memória lenta do RTC.
  memset(RTC_SLOW_MEM, 0, 8192);
  // Aqui defino os pinos que irei manipular mediante o ULP.
  const gpio_num_t lcdPINX = PINO_EIXO_X;
  const gpio_num_t lcdPINY = PINO_EIXO_Y;
  const int lcdPWMPINX = RTCIO_GPIO13_CHANNEL + 14;
  const int lcdPWMPINY = RTCIO_GPIO14_CHANNEL + 14;

  // GPIO32 initialization (set to output and initial value is 0)
  rtc_gpio_init(lcdPINX);
  rtc_gpio_init(lcdPINY);
  rtc_gpio_set_direction(lcdPINX, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_direction(lcdPINY, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(lcdPINX, 0);
  rtc_gpio_set_level(lcdPINY, 0);
  
  // Neste segmento de código definimos o programa que vai executar e simular
  // um sinal PWM nos dois pinos de RTC enquanto estamos em DEEPSLEEP.
  const ulp_insn_t  ulp_prog[] = {
    M_LABEL(1),
      I_WR_REG(RTC_GPIO_OUT_REG, lcdPWMPINX, lcdPWMPINX, 1),
      I_WR_REG(RTC_GPIO_OUT_REG, lcdPWMPINY, lcdPWMPINY, 1), // on
      I_DELAY(12800),
      I_WR_REG(RTC_GPIO_OUT_REG, lcdPWMPINX, lcdPWMPINX, 0),
      I_WR_REG(RTC_GPIO_OUT_REG, lcdPWMPINY, lcdPWMPINY, 0), // off
      I_DELAY(12800),
    M_BX(1),
  };
  // Run ULP program
  size_t size = sizeof(ulp_prog) / sizeof(ulp_insn_t);
  ulp_process_macros_and_load(0, ulp_prog, &size);
  ulp_run(0);
  esp_sleep_enable_ulp_wakeup();
}
#endif

/*
---------------
| Observações |
---------------
(i) Pode-ser que haja problemas ao apertar vários botões ao mesmo tempo na configuração atual.
(ii) Deveria mudar a resolução dos meus temporizadores a sendo de 10 bits. Assim faço uma contagem mais precisa mediante num_s.
*/

