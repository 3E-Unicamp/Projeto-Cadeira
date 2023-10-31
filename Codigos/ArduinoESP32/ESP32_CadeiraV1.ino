/*
-------------------------------------
| 3Cadeira (Arduino) - Protótipo V1 |
-------------------------------------
Programador: Melvin Gustavo Maradiaga Elvir
V0 e V1 - 23/10/2023

Neste código vamos implementar o algoritmo que vai realizar o controle mais formal da cadeira de rodas,
a depender do botão que for apertado, só que utilizando a Arduino framework.
A ideia é implementar o código de uma forma fácil de entender que permita, depois, a importação para o 
ESPIDF caso o objetivo for comercializar em menos escala o código desenvolvido. 

Neste programa iremos a utilizar as seguintes capacidades do ESP32:
- Modo de DEEPSLEEP
- Pinos de RTC (Pinos do Real Time Clock interno do ESP32: Pinos 25, 26 e 27)
- Pinos de saida PWM para gerar uma tensão de 0V - 3,3V (Pinos 18 e 19)
- Interrupção EXT1 do ESP32
- Timers internos do ESP32 (Timer0 e Timer1)

A ideia de funcionamento deste programa é a seguinte: 
Estou querendo usar o microcontrolador como uma interface de baixo consumo energético que detecte
qual botão foi apertado e encaminhe um sinal DC aos pinos do potenciometro (joystick) no controle
da cadeira de rodas. Para isso, vamos estar monitorando o estado de três botões externos ao ESP32

Fluxograma:

[Inicio]--->[Diretivos do preprocessador.]
            |
            |--> [Entro na função setup()]
            
[setup()]--->[]
             |
             |--> [Entro no LOOP]

[main()]--->[]
*/

/*
-------------------
| Pre-processador |
-------------------
Neste bloco de código defino algumas constantes que irei utilizar neste programa.
Caso eu precise modificar alguma funcionalidade do código (como, por exemplo, quais pinos
irão fazer o ESP32 sair do DEEPSLEEP), só precisamos alterar o valor de algumas constantes
do código.

(i) EXT1BITMASK:
(ii) PINO_EIXO_X:
(iii) PINO_EIXO_Y:
-- Constantes de Teste --
(iv) TESTE_SAIRDOSLEEP
*/
#define EXT1BITMASK (1ULL<<GPIO_NUM_25)|(1ULL<<GPIO_NUM_26)|(1ULL<<GPIO_NUM_27)
#define PINO_EIXO_X GPIO_NUM_13
#define PINO_EIXO_Y GPIO_NUM_14
#define PRESCALER_TIMERS 80
#define LEDC_TIMER_8_BIT 8
#define TEMPO_DEBOUNCE 10 //Em ms
//#define TESTE_SAIRDOSLEEP
#define TESTE_BOTAO

/*
---------------------------
| Declaração de Variáveis |
---------------------------
Neste segmento do código declaro algumas variáveis que irei utilizar no meu programa, que preciso
sejam globais para fins de uso no meu programa.

Primeiro declaro uma variável global que irá acompanhar qual botão apertei, para assim realizar a escrita 
correspondente. Sob a hipotese que o ESP32 com Arduino framework já implementa o FreeRTOS interno do 
microcontrolador, irei declarar as minhas variáveis globais com static.

Explicando cada variável declarada:
(i) num_botao: Esta variável vai armazenar o número de pino que fez o meu sistema acordar do DEEPSLEEP,
               por conta da interrupção EXT1.
(ii) timer_channels:
(iii) flag_deepSleep: Este é um tipo de variável especial que será armazenado na memória RTC do ESP32.
                      Assim, ao entrar em DEEPSLEEP o meu ESP32 ainda pode modificar esta variável, e
                      esse valor ainda existe após acordar o ESP32.
(iv) num_s: Esta variável será o contador que vou estar atualizando dentro do temporizador 0.
             Este contador precisa ser de 8bits para contar até os 3s desejados por mim.
             Como ela é manipulada dentro de uma SRI, defino ela como sendo volatil.
(v) *timer2_cfg:  
*/
static volatile uint16_t num_botao = 0;
enum channels_pwm{
  CHANNEL_Y,
  CHANNEL_X
};
enum temporizadores{
  TEMPORIZADOR_0,
  TEMPORIZADOR_1,
  TEMPORIZADOR_2
};
RTC_DATA_ATTR int flag_deepSleep = 0;
volatile uint16_t num_s = 0;
volatile uint16_t num_t1s = 0;
volatile uint16_t num_t2s = 0;
hw_timer_t *timer0_cfg = NULL;
hw_timer_t *timer1_cfg = NULL;
hw_timer_t *timer2_cfg = NULL;

/*
-------------------------------------
| SRIs (Sub-rotinas de Interrupção) |
-------------------------------------
*/
void IRAM_ATTR Timer0_ISR(){
  num_s++;
}

void IRAM_ATTR Timer1_ISR(){
  num_t1s++;
}

void IRAM_ATTR Timer2_ISR(){
  num_t2s++;
}

void IRAM_ATTR GPIO25_ISR(){
  num_botao = GPIO_NUM_25;
  num_s = 0;
}

void IRAM_ATTR GPIO26_ISR(){
  num_botao = GPIO_NUM_26;
  num_s = 0;
}

void IRAM_ATTR GPIO27_ISR(){
  num_botao = GPIO_NUM_27;
  num_s = 0;
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
  timerAlarmWrite(timer0_cfg, 15000, true);
  timer1_cfg = timerBegin(TEMPORIZADOR_1, PRESCALER_TIMERS, true);
  timerAttachInterrupt(timer1_cfg, &Timer1_ISR, true);
  timerAlarmWrite(timer1_cfg, 15000, true);
  timer2_cfg = timerBegin(TEMPORIZADOR_2, PRESCALER_TIMERS, true);
  timerAttachInterrupt(timer2_cfg, &Timer2_ISR, true);
  timerAlarmWrite(timer2_cfg, 15000, true);
  
  /*
  if(flag_deepSleep){
  timerAlarmEnable(timer0_cfg);
  }
  */
  if(flag_deepSleep){
  timerAlarmEnable(timer1_cfg);
  timerAlarmEnable(timer0_cfg);
  timerAlarmEnable(timer2_cfg);
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
    attachInterrupt(GPIO_NUM_25, GPIO25_ISR, FALLING);
    attachInterrupt(GPIO_NUM_26, GPIO26_ISR, FALLING);
    attachInterrupt(GPIO_NUM_27, GPIO27_ISR, FALLING);
  }
  // Aqui defino que a interrupção EXT1 vai acordar o meu ESP32 do sleep.
  esp_sleep_enable_ext1_wakeup(EXT1BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
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
    esp_deep_sleep_start();
  } else {
    if(num_s > 300){
      flag_deepSleep = 0;
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
    case 25:
      ledcWrite(CHANNEL_X, 255);
      ledcWrite(CHANNEL_Y, 125);
      #ifdef TESTE_BOTAO
        Serial.println("Modo 1");
        Serial.print("Aumento s:");
        Serial.println(num_t1s);
        Serial.print("Aumento s:");
        Serial.println(num_s);
        Serial.print("Aumento s:");
        Serial.println(num_t2s);
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
        Serial.println("Modo 2");
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

