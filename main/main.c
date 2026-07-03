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
#define MOTOR_ESTEIRA_PIN      GPIO_NUM_22 // ie/fe - rele do motor da esteira

#define DEBOUNCE_STABLE_READS 8  // leituras consecutivas iguais para validar (~80ms a 10ms/loop)
// Duracao do pulso de cada braco (solenoide). Cada braco pode ter um tempo
// proprio: o medio/grande costumam precisar de mais tempo que o pequeno.
#define PULSE_BRACO_P_MS 300     // braco pequeno
#define PULSE_BRACO_M_MS 500     // braco medio (mais tempo que o pequeno)
#define PULSE_BRACO_G_MS 700     // braco grande
// Tempo que a esteira gira ate a peca chegar ao braco correspondente. Cada
// tamanho tem seu proprio tempo, pois cada braco fica numa posicao diferente:
//   so sp          -> pequeno
//   sp + sm        -> medio
//   sp + sm + sg   -> grande
#define ESTEIRA_P_MS 3000 // pequeno (braco mais proximo)
#define ESTEIRA_M_MS 4000 // medio
#define ESTEIRA_G_MS 5000 // grande (braco mais distante)

void default_action(const Event *event) {
  SUP_DEBUG_PRINT("Action for %s event '%s'\n",
                  event->kind == CONTROLLABLE ? "CONTROLLABLE"
                                              : "UNCONTROLLABLE",
                  event->name);
}

// --- Saidas pulsadas: solenoides dos cilindros (mono-estaveis, retorno por mola) e sinaleira ---

typedef struct {
  gpio_num_t pin;
  bool active;
  TickType_t start_tick;
  uint32_t duration_ms; // duracao propria de cada saida temporizada
} PulseOutput;

// A esteira e tratada como uma saida temporizada (como os solenoides), porem
// com duracao propria (ESTEIRA_P/M/G_MS) bem maior que o pulso do braco.
enum { PULSE_BRACO_P, PULSE_BRACO_M, PULSE_BRACO_G, PULSE_ESTEIRA, PULSE_COUNT };

static PulseOutput pulse_outputs[PULSE_COUNT] = {
  [PULSE_BRACO_P] = {.pin = SOLENOIDE_BRACO_P_PIN, .duration_ms = PULSE_BRACO_P_MS},
  [PULSE_BRACO_M] = {.pin = SOLENOIDE_BRACO_M_PIN, .duration_ms = PULSE_BRACO_M_MS},
  [PULSE_BRACO_G] = {.pin = SOLENOIDE_BRACO_G_PIN, .duration_ms = PULSE_BRACO_G_MS},
  // duration_ms da esteira e definido em tempo de execucao conforme o tamanho
  // classificado (ESTEIRA_P/M/G_MS); aqui fica o valor do menor como padrao.
  [PULSE_ESTEIRA] = {.pin = MOTOR_ESTEIRA_PIN,     .duration_ms = ESTEIRA_P_MS},
};

void start_pulse(PulseOutput *pulse) {
  gpio_set_level(pulse->pin, 1);
  pulse->active = true;
  pulse->start_tick = xTaskGetTickCount();
}

void update_pulses(void) {
  for (int i = 0; i < PULSE_COUNT; i++) {
    PulseOutput *pulse = &pulse_outputs[i];
    if (pulse->active &&
        (xTaskGetTickCount() - pulse->start_tick) >= pdMS_TO_TICKS(pulse->duration_ms)) {
      gpio_set_level(pulse->pin, 0);
      pulse->active = false;
      if (i == PULSE_ESTEIRA) {
        // Fim do tempo da esteira: sinaliza 'fe' ao supervisor. A cascata de
        // eventos controlaveis dispara entao o braco (iBp/iBm/iBg) correspondente
        // ao tamanho que foi contado quando a peca entrou.
        trigger_event(&fe);
      }
    }
  }
}

void action_iBp(const Event *event) {
  default_action(event);
  start_pulse(&pulse_outputs[PULSE_BRACO_P]);
}

void action_iBm(const Event *event) {
  default_action(event);
  start_pulse(&pulse_outputs[PULSE_BRACO_M]);
}

void action_iBg(const Event *event) {
  default_action(event);
  start_pulse(&pulse_outputs[PULSE_BRACO_G]);
}

void action_ie(const Event *event) {
  default_action(event);
  // Liga a esteira por tempo determinado; update_pulses() a desliga sozinha
  // apos o tempo do tamanho classificado (nunca fica ligada indefinidamente).
  // O duration_ms ja foi ajustado em poll_input antes deste trigger.
  start_pulse(&pulse_outputs[PULSE_ESTEIRA]);
}

void action_fe(const Event *event) {
  default_action(event);
  // Garante a esteira desligada. O desligamento normal ja e feito pelo
  // temporizador em update_pulses(); aqui e redundante/seguranca.
  gpio_set_level(MOTOR_ESTEIRA_PIN, 0);
}

// --- Entradas com debounce: sensores de classificacao e fins de curso ---

typedef struct {
  gpio_num_t pin;
  Event *event;
  int last_level;
  int candidate_level;
  int stable_count;
  uint32_t belt_ms; // >0 nos sensores de classificacao: tempo da esteira p/ aquele tamanho
} DebouncedInput;

enum { IN_SP, IN_SM, IN_SG, IN_SMET, IN_FBP, IN_FBM, IN_FBG, IN_BOTAO, IN_COUNT };

static DebouncedInput debounced_inputs[IN_COUNT] = {
  [IN_SP]   = {.pin = SENSOR_PEQUENO_PIN,    .event = &sp,   .belt_ms = ESTEIRA_P_MS},
  [IN_SM]   = {.pin = SENSOR_MEDIO_PIN,      .event = &sm,   .belt_ms = ESTEIRA_M_MS},
  [IN_SG]   = {.pin = SENSOR_GRANDE_PIN,     .event = &sg,   .belt_ms = ESTEIRA_G_MS},
  [IN_SMET] = {.pin = SENSOR_METAL_PIN,      .event = &Smet},
  [IN_FBP]  = {.pin = FIM_CURSO_BRACO_P_PIN, .event = &fBp},
  [IN_FBM]  = {.pin = FIM_CURSO_BRACO_M_PIN, .event = &fBm},
  [IN_FBG]  = {.pin = FIM_CURSO_BRACO_G_PIN, .event = &fBg},
  [IN_BOTAO] = {.pin = BOTAO_PIN,            .event = &botao},
};

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
    if (input->candidate_level == 1) {
      trigger_event(input->event);
      if (input->belt_ms > 0) {
        // Sensor de classificacao. Define o tempo que a esteira deve girar ate
        // o braco correspondente ao tamanho.
        PulseOutput *belt = &pulse_outputs[PULSE_ESTEIRA];
        if (!belt->active) {
          // Inicio de um novo ciclo: primeiro feixe rompido (normalmente sp)
          // liga a esteira com o tempo daquele tamanho. trigger_event(&ie) so
          // tem efeito se 'ie' estiver habilitado (maquina armada, sem ciclo em
          // andamento); caso contrario e ignorado.
          belt->duration_ms = input->belt_ms;
          trigger_event(&ie);
        } else if (input->belt_ms > belt->duration_ms) {
          // Peca maior do que o detectado ate agora (ex.: sm/sg logo apos sp):
          // estende o tempo da esteira ate o braco correto, sem reiniciar o ciclo.
          belt->duration_ms = input->belt_ms;
        }
      }
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
  set_event_action(&ie, action_ie);
  set_event_action(&fe, action_fe);
  set_event_action(&iBp, action_iBp);
  set_event_action(&iBm, action_iBm);
  set_event_action(&iBg, action_iBg);
  setup_gpio();
}

void loop(void) {
  for (int i = 0; i < IN_COUNT; i++) {
    poll_input(&debounced_inputs[i]);
  }
  update_pulses();
}

int app_main(void) {
  setup();
  while (1) {
    loop();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return 0;
}
