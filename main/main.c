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
#define ESTEIRA_PASSO 2000

// --- Memoria da esteira (registrador de deslocamento) ---
// Cada posicao guarda o tipo da peca naquela estacao fisica:
//   esteira_mem[0] = posicao 1 (sensor / entrada)
//   esteira_mem[1] = posicao 2 (braco pequeno)
//   esteira_mem[2] = posicao 3 (braco medio)
//   esteira_mem[3] = posicao 4 (braco grande)
// A cada passo da esteira o vetor desloca uma casa para frente.
typedef enum { VAZIO = 0, PEQUENA = 1, MEDIA = 2, GRANDE = 3 } TipoPeca;
#define NUM_POSICOES 4
static TipoPeca esteira_mem[NUM_POSICOES] = {VAZIO};

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

enum { IN_SP, IN_SM, IN_SG, IN_SMET, IN_FBP, IN_FBM, IN_FBG, IN_BOTAO, IN_COUNT };

static DebouncedInput debounced_inputs[IN_COUNT] = {
  [IN_SP]   = {.pin = SENSOR_PEQUENO_PIN},
  [IN_SM]   = {.pin = SENSOR_MEDIO_PIN},
  [IN_SG]   = {.pin = SENSOR_GRANDE_PIN},
  [IN_SMET] = {.pin = SENSOR_METAL_PIN},
  [IN_FBP]  = {.pin = FIM_CURSO_BRACO_P_PIN},
  [IN_FBM]  = {.pin = FIM_CURSO_BRACO_M_PIN},
  [IN_FBG]  = {.pin = FIM_CURSO_BRACO_G_PIN},
  [IN_BOTAO] = {.pin = BOTAO_PIN,            .event = &botao},
};

// Classifica a peca que esta na posicao 1 (sensor) pelos feixes rompidos.
// Robusto tanto se a peca grande rompe os 3 feixes (cumulativo) quanto se cada
// tamanho rompe apenas o seu: prioriza o maior feixe rompido.
static TipoPeca classifica_sensor(void) {
  if (debounced_inputs[IN_SG].last_level == 1) return GRANDE;
  if (debounced_inputs[IN_SM].last_level == 1) return MEDIA;
  if (debounced_inputs[IN_SP].last_level == 1) return PEQUENA;
  return VAZIO;
}

// Chamada uma vez a cada parada (borda andar->parar): desloca o registrador uma
// casa e aciona os bracos das pecas que chegaram a sua estacao de descarte.
static void on_belt_stop(void) {
  // desloca uma posicao para frente (a peca do sensor vai para o braco P, etc.)
  for (int i = NUM_POSICOES - 1; i > 0; i--) {
    esteira_mem[i] = esteira_mem[i - 1];
  }
  esteira_mem[0] = VAZIO;

  // Cada braco derruba a peca do seu tamanho quando ela chega na sua posicao.
  // As checagens sao independentes: se duas pecas chegam juntas, os dois bracos
  // sao acionados na mesma parada.
  if (esteira_mem[1] == PEQUENA) {
    start_pulse(&pulse_outputs[PULSE_BRACO_P]);
    esteira_mem[1] = VAZIO;
  }
  if (esteira_mem[2] == MEDIA) {
    start_pulse(&pulse_outputs[PULSE_BRACO_M]);
    esteira_mem[2] = VAZIO;
  }
  if (esteira_mem[3] == GRANDE) {
    start_pulse(&pulse_outputs[PULSE_BRACO_G]);
    esteira_mem[3] = VAZIO;
  }
}

// Alterna a esteira entre andar um passo e ficar parada pelo tempo de um passo.
// A cada transicao andar->parar avanca o registrador (on_belt_stop). Enquanto
// parada, espelha o sensor na posicao de entrada (esteira_mem[0]).
void update_belt_step(void) {
  if (!belt_stepping) {
    return;
  }
  if ((xTaskGetTickCount() - belt_step_tick) >= pdMS_TO_TICKS(ESTEIRA_PASSO)) {
    belt_moving = !belt_moving;
    gpio_set_level(MOTOR_ESTEIRA_PIN, belt_moving ? 1 : 0);
    belt_step_tick = xTaskGetTickCount();
    if (!belt_moving) {
      // acabou de parar: avanca a memoria e aciona os bracos
      on_belt_stop();
    }
  }
  if (!belt_moving) {
    // parada: registra na posicao 1 a peca que esta sob o sensor
    esteira_mem[0] = classifica_sensor();
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
    // Os sensores de classificacao/fim de curso apenas mantem last_level, que e
    // lido por classifica_sensor().
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
