/**
 * @file main.c
 *
 * @brief Main file for the conveyor belt material-sorting supervisor project
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "events.h"
#include "event_handler.h"
#include "driver/gpio.h"

// Sensores de classificacao / fim de curso (entradas, ativos em nivel alto)
#define SENSOR_PEQUENO_PIN     GPIO_NUM_32 // sp   - barreira optica: objeto pequeno
#define SENSOR_MEDIO_PIN       GPIO_NUM_33 // sm   - barreira optica: objeto medio
#define SENSOR_GRANDE_PIN      GPIO_NUM_25 // sg   - barreira optica: objeto grande
#define SENSOR_METAL_PIN       GPIO_NUM_26 // Smet - sensor indutivo: objeto metalico
#define FIM_CURSO_BRACO_P_PIN  GPIO_NUM_27 // fBp  - cilindro pequeno em repouso
#define FIM_CURSO_BRACO_M_PIN  GPIO_NUM_14 // fBm  - cilindro medio em repouso
#define FIM_CURSO_BRACO_G_PIN  GPIO_NUM_4  // fBg  - cilindro grande em repouso
#define BOTAO_PIN              GPIO_NUM_23 // botao - botao de controle

// Atuadores (saidas)
#define SOLENOIDE_BRACO_P_PIN  GPIO_NUM_18 // iBp - valvula do cilindro pequeno
#define SOLENOIDE_BRACO_M_PIN  GPIO_NUM_19 // iBm - valvula do cilindro medio
#define SOLENOIDE_BRACO_G_PIN  GPIO_NUM_21 // iBg - valvula do cilindro grande
#define SOLENOIDE_BRACO_G_VOLTA GPIO_NUM_5  // iBgV - valvula do cilindro grande (volta)
#define MOTOR_ESTEIRA_PIN      GPIO_NUM_22 // ie/fe - rele do motor da esteira

#define DEBOUNCE_STABLE_READS 8  // leituras consecutivas iguais para validar (~80ms a 10ms/loop)
// Duracao do pulso de cada braco (solenoide). Cada braco pode ter um tempo
// proprio: o medio/grande costumam precisar de mais tempo que o pequeno.
// Precisa ser menor que ESTEIRA_PASSO para o braco empurrar e recolher durante
// a parada, antes da esteira voltar a andar.
#define PULSE_BRACO_P_MS 300     // braco pequeno
#define PULSE_BRACO_M_MS 500     // braco medio (mais tempo que o pequeno)
#define PULSE_BRACO_G_MS 700     // braco grande

// Passo da esteira: anda por ESTEIRA_PASSO e em seguida fica parada por
// ESTEIRA_PASSO, repetindo. Cada parada equivale a "uma casa" do vetor de memoria.
// Com o passo reduzido pela metade, cada casa cobre metade da distancia, entao
// cada braco fica a DUAS casas do anterior (uma casa intermediaria no meio).
#define ESTEIRA_PASSO 1000

// Imprime o vetor da esteira a cada parada na serial. Comente para desligar.
#define DEBUG_ESTEIRA 1

// --- Memoria da esteira (registrador de deslocamento) ---
// Cada posicao guarda o tipo da peca naquela casa fisica. Como cada braco fica a
// duas casas do anterior, ha uma casa intermediaria entre as estacoes uteis:
//   esteira_mem[0] = sensor pequeno  - TODA peca para exatamente aqui
//   esteira_mem[1] = (intermediaria)
//   esteira_mem[2] = braco pequeno
//   esteira_mem[3] = (intermediaria)
//   esteira_mem[4] = braco medio
//   esteira_mem[5] = (intermediaria)
//   esteira_mem[6] = braco grande
// A cada passo da esteira o vetor desloca uma casa para frente.
typedef enum { VAZIO = 0, PEQUENA = 1, MEDIA = 2, GRANDE = 3 } TipoPeca;
#define POS_SENSOR   0
#define POS_BRACO_P  2
#define POS_BRACO_M  4
#define POS_BRACO_G  6
#define NUM_POSICOES 7
static TipoPeca esteira_mem[NUM_POSICOES] = {VAZIO};

static uint32_t passo_num = 0; // conta as paradas desde o inicio

// Latch do sensor medio. O sensor medio fica ANTES do sensor pequeno e FORA de um
// ponto de parada, entao a peca so passa por ele enquanto a esteira anda (pulso
// rapido). A interrupcao marca aqui que a peca que esta chegando ao sensor pequeno
// e media. A classificacao final acontece na parada (classifica_no_sensor), e o
// latch e consumido/limpo la.
static volatile bool medio_visto = false;

// Simbolo de cada tipo de peca para impressao.
static char simbolo_peca(TipoPeca t) {
  switch (t) {
    case PEQUENA: return 'P';
    case MEDIA:   return 'M';
    case GRANDE:  return 'G';
    default:      return '.';
  }
}

// Imprime na serial o estado atual do vetor da esteira.
//   idx0=sensor  idx2=braco P  idx4=braco M  idx6=braco G  (impares=intermediarias)
void print_belt(void) {
#ifdef DEBUG_ESTEIRA
  printf("Passo %3u | esteira: [ ", (unsigned) passo_num);
  for (int i = 0; i < NUM_POSICOES; i++) {
    printf("%c%s", simbolo_peca(esteira_mem[i]), (i < NUM_POSICOES - 1) ? " | " : " ");
  }
  printf("]  (idx0=sensor, idx2=P, idx4=M, idx6=G)\n");
#endif
}

void default_action(const Event *event) {
  SUP_DEBUG_PRINT("Action for %s event '%s'\n",
                  event->kind == CONTROLLABLE ? "CONTROLLABLE"
                                              : "UNCONTROLLABLE",
                  event->name);
}

// --- Saidas pulsadas: solenoides dos cilindros (mono-estaveis, retorno por mola) ---

typedef struct {
  gpio_num_t pin;
  bool active;
  TickType_t start_tick;
  uint32_t duration_ms; // duracao propria de cada saida temporizada
} PulseOutput;

enum { PULSE_BRACO_P, PULSE_BRACO_M, PULSE_BRACO_G, PULSE_BRACO_G_VOLTA, PULSE_COUNT };

// --- Modo passo-a-passo da esteira ---
// A partir do aperto do botao, a esteira anda por ESTEIRA_PASSO e em seguida
// fica parada por ESTEIRA_PASSO, repetindo indefinidamente.
static bool belt_stepping = false;      // true depois que o botao foi apertado
static bool belt_moving = false;        // fase atual: true=andando, false=parada
static TickType_t belt_step_tick = 0;   // instante em que a fase atual comecou

static PulseOutput pulse_outputs[PULSE_COUNT] = {
  [PULSE_BRACO_P] = {.pin = SOLENOIDE_BRACO_P_PIN, .duration_ms = PULSE_BRACO_P_MS},
  [PULSE_BRACO_M] = {.pin = SOLENOIDE_BRACO_M_PIN, .duration_ms = PULSE_BRACO_M_MS},
  [PULSE_BRACO_G] = {.pin = SOLENOIDE_BRACO_G_PIN, .duration_ms = PULSE_BRACO_G_MS},
  [PULSE_BRACO_G_VOLTA] = {.pin = SOLENOIDE_BRACO_G_VOLTA, .duration_ms = PULSE_BRACO_G_MS},
};

void start_pulse(PulseOutput *pulse) {
  gpio_set_level(pulse->pin, 1);
  pulse->active = true;
  pulse->start_tick = xTaskGetTickCount();
}

// Desliga cada solenoide de braco apos o seu tempo de pulso; a mola recolhe o
// cilindro. Varios bracos podem estar pulsando ao mesmo tempo (saidas independentes).
void update_pulses(void) {
  for (int i = 0; i < PULSE_COUNT; i++) {
    PulseOutput *pulse = &pulse_outputs[i];
    if (pulse->active &&
        (xTaskGetTickCount() - pulse->start_tick) >= pdMS_TO_TICKS(pulse->duration_ms)) {
      gpio_set_level(pulse->pin, 0);
      pulse->active = false;
    }
  }
}

// --- Entradas com debounce: sensores de classificacao e fins de curso ---

typedef struct {
  gpio_num_t pin;
  Event *event;     // != NULL apenas para entradas que disparam evento (botao)
  int last_level;
  int candidate_level;
  int stable_count;
} DebouncedInput;

// Os sensores de classificacao (pequeno/medio/grande) nao entram aqui: o medio e
// tratado por interrupcao (sensor_medio_isr) e o pequeno e lido por nivel na
// parada (classifica_no_sensor).
enum { IN_SMET, IN_FBP, IN_FBM, IN_FBG, IN_BOTAO, IN_COUNT };

static DebouncedInput debounced_inputs[IN_COUNT] = {
  [IN_SMET] = {.pin = SENSOR_METAL_PIN},
  [IN_FBP]  = {.pin = FIM_CURSO_BRACO_P_PIN},
  [IN_FBM]  = {.pin = FIM_CURSO_BRACO_M_PIN},
  [IN_FBG]  = {.pin = FIM_CURSO_BRACO_G_PIN},
  [IN_BOTAO] = {.pin = BOTAO_PIN,            .event = &botao},
};

// Interrupcao do sensor MEDIO (borda de subida). O sensor medio fica antes do
// sensor pequeno e FORA de um ponto de parada: a peca so passa por ele enquanto a
// esteira anda (pulso rapido), por isso precisa de interrupcao. Aqui apenas
// marca-se que a peca que esta chegando ao sensor pequeno e media; a decisao final
// (media x pequena x vazio) acontece na parada, em classifica_no_sensor().
// O sensor grande esta quebrado e e ignorado.
static void IRAM_ATTR sensor_medio_isr(void *arg) {
  (void) arg;
  // confirma nivel alto: filtra glitches e a interrupcao espuria de boot
  if (gpio_get_level(SENSOR_MEDIO_PIN) == 1) {
    medio_visto = true;
  }
}

// Le a peca que esta parada EXATAMENTE sobre o sensor pequeno (posicao 1 do
// vetor). Toda peca para nesse ponto antes de seguir para o braco pequeno. O tipo
// vem do latch do sensor medio: se o feixe medio foi rompido enquanto a peca se
// aproximava -> media; caso contrario -> pequena. Sem peca sobre o sensor -> vazio.
// Le o NIVEL (presenca), nao bordas: assim o pulso longo da media nao vira uma
// pequena fantasma.
static TipoPeca classifica_no_sensor(void) {
  if (gpio_get_level(SENSOR_PEQUENO_PIN) != 1) {
    return VAZIO;
  }
  return medio_visto ? MEDIA : PEQUENA;
}

// Chamada uma vez a cada parada (borda andar->parar): desloca o registrador uma
// casa e aciona os bracos das pecas que chegaram a sua estacao de descarte.
static void on_belt_stop(void) {
  // desloca uma posicao para frente (a peca do braco P vai para o braco M, etc.)
  for (int i = NUM_POSICOES - 1; i > 0; i--) {
    esteira_mem[i] = esteira_mem[i - 1];
  }
  // A peca que esta parada sobre o sensor pequeno entra na posicao do sensor
  // (index 0). Como cada braco fica a duas casas de distancia, ela leva DOIS
  // passos ate chegar no braco pequeno, quatro ate o medio e seis ate o grande.
  esteira_mem[POS_SENSOR] = classifica_no_sensor();
  medio_visto = false; // latch do sensor medio consumido; pronto para a proxima peca

  // Mostra o vetor deste passo ANTES de derrubar, para ver a peca chegando na
  // posicao do braco.
  passo_num++;
  print_belt();

  // Cada braco derruba a peca do seu tamanho quando ela chega na sua posicao.
  // As checagens sao independentes: se duas pecas chegam juntas, os dois bracos
  // sao acionados na mesma parada.
  if (esteira_mem[POS_BRACO_P] == PEQUENA) {
    start_pulse(&pulse_outputs[PULSE_BRACO_P]);
    esteira_mem[POS_BRACO_P] = VAZIO;
  }
  if (esteira_mem[POS_BRACO_M] == MEDIA) {
    start_pulse(&pulse_outputs[PULSE_BRACO_M]);
    esteira_mem[POS_BRACO_M] = VAZIO;
  }
  if (esteira_mem[POS_BRACO_G] == GRANDE) {
    start_pulse(&pulse_outputs[PULSE_BRACO_G]);
    esteira_mem[POS_BRACO_G] = VAZIO;
  }
}

// Alterna a esteira entre andar um passo e ficar parada pelo tempo de um passo.
// A cada transicao andar->parar avanca o registrador (on_belt_stop), que le a
// peca parada sobre o sensor pequeno e a coloca na posicao 1.
void update_belt_step(void) {
  if (!belt_stepping) {
    return;
  }
  if ((xTaskGetTickCount() - belt_step_tick) >= pdMS_TO_TICKS(ESTEIRA_PASSO)) {
    belt_moving = !belt_moving;
    gpio_set_level(MOTOR_ESTEIRA_PIN, belt_moving ? 1 : 0);
    belt_step_tick = xTaskGetTickCount();
    if (!belt_moving) {
      // acabou de parar: avanca a memoria, imprime o vetor e aciona os bracos
      on_belt_stop();
    }
  }
}

// Ao apertar o botao, inicia o modo passo-a-passo: a esteira comeca a andar
// (primeiro passo) e a partir dai alterna andar/parar a cada ESTEIRA_PASSO.
void action_botao(const Event *event) {
  default_action(event);
  belt_stepping = true;
  belt_moving = true;
  gpio_set_level(MOTOR_ESTEIRA_PIN, 1);
  belt_step_tick = xTaskGetTickCount();
}

void poll_input(DebouncedInput *input) {
  int level = gpio_get_level(input->pin);

  if (level == input->candidate_level) {
    if (input->stable_count < DEBOUNCE_STABLE_READS) {
      input->stable_count++;
    }
  } else {
    // reading changed before it was confirmed stable: restart the count
    input->candidate_level = level;
    input->stable_count = 0;
  }

  if (input->stable_count == DEBOUNCE_STABLE_READS &&
      input->candidate_level != input->last_level) {
    input->last_level = input->candidate_level;
    // Entradas com evento (botao) sinalizam ao supervisor na borda de subida.
    // As demais (metal/fim de curso) apenas mantem last_level para consulta.
    if (input->candidate_level == 1 && input->event != NULL) {
      trigger_event(input->event);
    }
  }
}

void setup_gpio(void) {
  uint64_t output_mask = (1ULL << SOLENOIDE_BRACO_P_PIN) |
                          (1ULL << SOLENOIDE_BRACO_M_PIN) |
                          (1ULL << SOLENOIDE_BRACO_G_PIN) |
                          (1ULL << MOTOR_ESTEIRA_PIN);
  gpio_config_t output_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = output_mask,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE,
  };
  gpio_config(&output_conf);

  uint64_t input_mask = 0;
  for (int i = 0; i < IN_COUNT; i++) {
    input_mask |= (1ULL << debounced_inputs[i].pin);
  }
  // sensores ficam em repouso em nivel baixo (pull-down) e sobem ao detectar o objeto/curso
  gpio_config_t input_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = input_mask,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE,
  };
  gpio_config(&input_conf);

  // Sensores de classificacao pequeno e medio. O sensor grande esta quebrado, por
  // isso nao e configurado/tratado por enquanto.
  //  - pequeno: fica EXATAMENTE num ponto de parada; a peca para sobre ele. Basta
  //             ler o NIVEL na parada (classifica_no_sensor), sem interrupcao.
  //  - medio:   fica FORA de um ponto de parada; a peca so passa por ele andando,
  //             entao precisa de interrupcao na borda de subida (sensor_medio_isr).
  gpio_config_t sensor_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = (1ULL << SENSOR_PEQUENO_PIN) | (1ULL << SENSOR_MEDIO_PIN),
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE,
  };
  gpio_config(&sensor_conf);

  // So o sensor medio usa interrupcao. Registra o handler primeiro e so entao
  // habilita a borda de subida, para evitar a interrupcao espuria que ocorre
  // quando a interrupcao ja esta habilitada antes do handler existir.
  gpio_install_isr_service(0);
  gpio_isr_handler_add(SENSOR_MEDIO_PIN, sensor_medio_isr, NULL);
  gpio_set_intr_type(SENSOR_MEDIO_PIN, GPIO_INTR_POSEDGE);
}

void setup(void) {
  printf("Start supervisor!\n\n");
  // O acionamento dos bracos agora e feito diretamente pela memoria da esteira
  // (esteira_mem) em on_belt_stop(); o supervisor so trata o 'botao'.
  set_event_action(&botao, action_botao);
  setup_gpio();
}

void loop(void) {
  for (int i = 0; i < IN_COUNT; i++) {
    poll_input(&debounced_inputs[i]);
  }
  update_pulses();
  update_belt_step();
}

int app_main(void) {
  setup();
  while (1) {
    loop();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return 0;
}
