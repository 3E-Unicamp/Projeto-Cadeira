/*
-------------------------------------
| 3Cadeira (Arduino) - Protótipo V0 |
-------------------------------------
Programador: Melvin Gustavo Maradiaga Elvir

Neste código vamos implementar o algoritmo que vai realizar o controle mais formal da cadeira de rodas,
a depender do botão que for apertado, só que utilizando a Arduino framework.

A ideia é implementar os códigos de uma forma fácil de entender que permita, depois, a importação para o 
ESPIDF caso o objetivo for comercializar em menos escala o código desenvolvido. 

A ideia é implementar os códigos de uma forma fácil de entender que permita, depois, a importação para o 
ESPIDF.

Neste programa iremos a utilizar as seguintes capacidades do ESP32:
- Modo de DEEPSLEEP
- Pinos de RTC (Pinos do Real Time Clock interno do ESP32)
- Pinos de saida analógica para gerar uma tensão de 0V - 3,3V
- Interrupção EXT1 do ESP32

Os pinos de RTC que irei utilizar para acordar o ESP32 são os pinos 25, 26 e 27.

A ideia de funcionamento deste programa é a seguinte: 
Estou querendo usar o microcontrolador como uma interface de baixo consumo energético que detecte
qual botão foi apertado e encaminhe um sinal DC aos pinos do potenciometro (joystick) no controle
da cadeira de rodas.

Fluxograma:

[Inicio]--->[Diretivos do preprocessador.]
            |
            |--> [Entro na função setup()]
            
[setup()]--->[]
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
*/
#define ext1_bitmask (1ULL<<GPIO_NUM_25)|(1ULL<<GPIO_NUM_26)|(1ULL<<GPIO_NUM_27)

/*
---------------------------
| Declaração de Variáveis |
---------------------------
Neste segmento do código declaro algumas variáveis que irei utilizar no meu programa, que preciso
sejam globais para fins de uso no meu programa.
Sob a hipotese que o ESP32 com Arduino framework já implementa o FreeRTOS interno do microcontrolador,
irei declarar as minhas variáveis globais com static.
*/

static uint16_t num_botao = 0;

void print_motivo_acordar(){
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason != ESP_SLEEP_WAKEUP_EXT1){
    Serial.println("Wakeup caused by ESP32 normal reset.\n");
    delay(1000);
  } else {
    uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
    Serial.println("Wakeup caused by external signal using RTC_IO\n");
    delay(1000);
    if(wakeup_pin_mask != 0){
      num_botao = __builtin_ffsll(wakeup_pin_mask) - 1;
      Serial.print("Wakeup from GPIO");
      Serial.println(num_botao);
      delay(1000);
    } 
  }
}


/*
--------------------------
| Função de Configuração |
--------------------------
Nesta função configuro os periféricos que irei utilizar neste programa. Dentro do Arduino IDE,
a função setup() contem códigos que serão executados uma única vez durante o funcionamento
do ESP32.

Como pretendo utilizar a interrupção ext1, note-se que precisarei primeiro configurar quais pinos
vou associar a essa interrupção.
*/
void setup() {
  // Coloco o delay neste segmento para permitir o Serial ser, de fato, ativado.
  Serial.begin(115200);
  delay(1000);

  // Aqui escrevo no monitor serial o motivo pelo qual meu ESP32 acordou.
  print_motivo_acordar();

  // Aqui defino que a interrupção EXT1 vai acordar o meu ESP32 do sleep.
  esp_sleep_enable_ext1_wakeup(ext1_bitmask ,ESP_EXT1_WAKEUP_ANY_HIGH);

  // Como o periferico utilizado para encaminhar as mensagens do serial demora vários ciclos de relógio
  // para encaminhar os caracteres, precisamos colocar esse delay para garantir que a mensagem seja encaminhada.
  // Neste caso, é ainda mais importante por conta do deep_sleep colocado logo depois.
  Serial.println("Entrando em modo deep sleep");
  delay(1000);
  esp_deep_sleep_start();
}

void loop() {
  delay(10);
}