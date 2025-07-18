#include "XMC1300.h"
#include "cy_utils.h"
#include "cybsp.h"
#include "cycfg_peripherals.h"
#include "cycfg_pins.h"
#include "xmc1_gpio.h"
#include "xmc_gpio.h"
#include "xmc_uart.h"
#include "xmc_usic.h"
#include <stdint.h>
#include <xmc_device.h>
#include <xmc_flash.h>
#include <xmc_scu.h>

#define ADRS0 2
#define ADRS1 3
#define ADRS2 4
#define ADRS3 5
#define FUNCTION 6
#define ORIGIN 7
#define DATA 8

#define MASTER 1
#define SLAVE 2

#define FLASH_SECTOR_ADDR 0x10010000 // Endereço de setor livre

#define TAMANHO_BUFFER 12
// Buffer de envio e recepção de dados
volatile uint8_t Rx_buffer[TAMANHO_BUFFER];
volatile uint8_t Rx_buffer_index = 0;
uint8_t Buffer_TX[TAMANHO_BUFFER] = {0};

typedef enum {
  PGM_ID = 0x01,
  PGM_BROADCAST_ID = 0x02,
  PGM_ID_RESPONSE = 0x03,
} PGM_DEVICE_ID_t;

// Flags de envio e recepção de dados
volatile bool recebendo = false;
volatile bool pacote_completo = false;
volatile bool pacote_obsoleto = false;
volatile bool cadastrado = true;
volatile bool enviar_pacote = false;
volatile bool esperando_ack = false;
volatile bool toggle_on = false;
volatile bool delete_on = false;
volatile bool broadcast_on = false;
volatile bool broadcast_validate = false;
// Variáveis do pacote
#define start_byte 0x7E
uint8_t UID0 = 0;
uint8_t UID1 = 0;
uint8_t UID2 = 0;
uint8_t UID3 = 0;
uint8_t Function = 0;
uint8_t origem_pacote = 0;
uint8_t dado_pacote = 0;
uint8_t checksum = 0;
#define stop_byte 0x81
#define ACK 0x06
uint8_t PGM_count = 1;
uint8_t ack_count = 0;

// Variáveis de tempo/delays
#define TIMEOUT_MAX 2000
uint32_t sem_comunicação = 0;
uint32_t hold_b1 = 0;
uint8_t Tempo_200ms = 10;
uint16_t Tempo_3000ms = 1000;
uint16_t Tempo_100ms = 100;
uint32_t tentando_cadastrar = 100;
uint8_t piscadas = 0;
volatile uint32_t systick = 0;
volatile uint32_t delay_tx = 0;
volatile bool acionar_200ms = false;
volatile bool acionar_3000ms = false;
volatile bool acionar_100ms = false;
volatile bool piscando = false;
volatile bool esperando_debounce = false;
volatile bool pb1_ultimo_estado = true;
volatile bool pb2_ultimo_estado = true;
volatile bool pb3_ultimo_estado = true;
volatile bool pb4_ultimo_estado = true;
volatile bool get_status = false;
// Estado dos botões
#define NUM_LEDS 5
uint8_t ligar_reles = 0;
uint8_t status_reles = 0;
uint8_t modulo_index = 1;
volatile bool led_envio = 0;
volatile bool led_cadastro = false;
volatile bool PGM_cadastrado[5] = {0, 0, 0, 0, 0};

typedef struct {
  XMC_GPIO_PORT_t *port;
  uint8_t pin;
  uint8_t blink_count;
  uint8_t blink_target;
  bool piscando;
} LED_t;

LED_t leds[NUM_LEDS] = {
    {LED_PB1_PORT, LED_PB1_PIN, 0, 0, false},
    {LED_PB2_PORT, LED_PB2_PIN, 0, 0, false},
    {LED_PB3_PORT, LED_PB3_PIN, 0, 0, false},
    {LED_PB4_PORT, LED_PB4_PIN, 0, 0, false},
    {LED_ST_PORT, LED_ST_PIN, 0, 0, false},
};

typedef struct {
  uint8_t numero;
  uint8_t UID0;
  uint8_t UID1;
  uint8_t UID2;
  uint8_t UID3;
} PGM_t;

PGM_t modulos[5] = {
    {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
};

// Máquina de estados
uint8_t estado = 0;

typedef struct {
  PGM_t modulos[5];
  uint8_t pgm_count;
  uint8_t numero_modulo;
  uint8_t pgm_cadastrado[5];
  uint8_t reservado[3]; // Padding opcional para alinhar em 4 bytes
} FlashData_t;

FlashData_t dados_flash;

void salvar_modulos_na_flash(void) {
  for (int i = 0; i < 5; i++) {
    dados_flash.modulos[i] = modulos[i];
  }

  dados_flash.pgm_count = PGM_count;

  for (int i = 0; i < 5; i++) {
    dados_flash.pgm_cadastrado[i] = PGM_cadastrado[i];
  }

  // Apaga setor onde os dados serão gravados
  XMC_FLASH_EraseSector((uint32_t *)FLASH_SECTOR_ADDR);

  // Grava os dados (usa ProgramPage que aceita até 256 bytes por vez)
  XMC_FLASH_ProgramPage((uint32_t *)FLASH_SECTOR_ADDR,
                        (uint32_t *)&dados_flash);
}

void carregar_modulos_da_flash(void) {
  const FlashData_t *ptr_flash = (const FlashData_t *)FLASH_SECTOR_ADDR;

  for (int i = 0; i < 5; i++) {
    if (ptr_flash->modulos[i].numero == 0xFF &&
        ptr_flash->modulos[i].UID0 == 0xFF &&
        ptr_flash->modulos[i].UID1 == 0xFF &&
        ptr_flash->modulos[i].UID2 == 0xFF &&
        ptr_flash->modulos[i].UID3 == 0xFF) {
      // Dados inválidos (apagados), inicializa com 0
      modulos[i].numero = 0;
      modulos[i].UID0 = 0;
      modulos[i].UID1 = 0;
      modulos[i].UID2 = 0;
      modulos[i].UID3 = 0;
    } else {
      // Dados válidos
      modulos[i] = ptr_flash->modulos[i];
    }
  }

  for (int i = 0; i < 5; i++) {
    if (ptr_flash->pgm_cadastrado[i] == 0xFF) {
      PGM_cadastrado[i] = 0;
    } else {
      PGM_cadastrado[i] = ptr_flash->pgm_cadastrado[i];
    }
  }

  // Verifica se o PGM_count está apagado
  if (ptr_flash->pgm_count == 0xFF) {
    PGM_count = 1;
  } else {
    PGM_count = ptr_flash->pgm_count;
  }
}

void montar_pacote(uint8_t size, uint8_t ID, uint8_t Addrs_1, uint8_t Addrs_2,
                   uint8_t Addrs_3, uint8_t Addrs_4, uint8_t function,
                   uint8_t origin, uint8_t data, volatile uint8_t *destino) {
  destino[0] = start_byte;
  destino[1] = size;
  destino[2] = ID;
  destino[3] = Addrs_1;
  destino[4] = Addrs_2;
  destino[5] = Addrs_3;
  destino[6] = Addrs_4;
  destino[7] = function;
  destino[8] = origin;
  destino[9] = data;

  destino[10] =
      ~(destino[0] ^ destino[1] ^ destino[2] ^ destino[3] ^ destino[4] ^
        destino[5] ^ destino[6] ^ destino[7] ^ destino[8] ^ destino[9]);

  destino[11] = stop_byte;
}

void enviar_ack() {

  montar_pacote(12, PGM_ID, modulos[PGM_count].UID0, modulos[PGM_count].UID1,
                modulos[PGM_count].UID2, modulos[PGM_count].UID3, 'A', 0x01,
                modulos[PGM_count].numero, Buffer_TX);

  XMC_GPIO_SetOutputLow(Bus_Controle_PORT, Bus_Controle_PIN);
  for (int i = 0; i < sizeof(Buffer_TX); i++) {

    XMC_UART_CH_Transmit(UART1_HW, Buffer_TX[i]);
  }

  while (!XMC_USIC_CH_TXFIFO_IsEmpty(UART1_HW))
    ;
  XMC_GPIO_SetOutputHigh(Bus_Controle_PORT, Bus_Controle_PIN);
}
// Rotina para receber dados
void USIC0_1_IRQHandler(void) {
  if (pacote_completo == false) {

    while (!XMC_USIC_CH_RXFIFO_IsEmpty(UART1_HW)) {

      uint8_t rx = XMC_UART_CH_GetReceivedData(UART1_HW);
      if (!recebendo) {
        if (rx == 0x7E) {
          recebendo = true;
          Rx_buffer_index = 0;
        }
      } else {
        if (rx == 0x81) {
          recebendo = false;
          if (Rx_buffer[ORIGIN] == SLAVE) {
            pacote_completo = true;
            sem_comunicação = 0;
          }

          Rx_buffer_index = 0;

          break;
        } else {

          if (Rx_buffer_index < TAMANHO_BUFFER) {
            Rx_buffer[Rx_buffer_index++] = rx;
          } else {
            recebendo = false;
            Rx_buffer_index = 0;
          }
        }
      }
    }

    if ((Rx_buffer[FUNCTION] == 'A') && pacote_completo &&
        (Rx_buffer[ADRS0] != 0 || Rx_buffer[ADRS1] != 0 || Rx_buffer[ADRS2] != 0 ||
         Rx_buffer[ADRS3] != 0) &&
        Rx_buffer[DATA] == ACK) {

      for (int i = 1; i < PGM_count; i++) {
        if (Rx_buffer[ADRS0] == modulos[i].UID0 &&
            Rx_buffer[ADRS1] == modulos[i].UID1 &&
            Rx_buffer[ADRS2] == modulos[i].UID2 &&
            Rx_buffer[ADRS3] == modulos[i].UID3) {

          cadastrado = true;
          break;
        }
      }

      if (PGM_count < 5 && !cadastrado) {
        broadcast_validate = true;
        PGM_cadastrado[PGM_count] = true;
        modulos[PGM_count].numero = PGM_count;
        modulos[PGM_count].UID0 = Rx_buffer[ADRS0];
        modulos[PGM_count].UID1 = Rx_buffer[ADRS1];
        modulos[PGM_count].UID2 = Rx_buffer[ADRS2];
        modulos[PGM_count].UID3 = Rx_buffer[ADRS3];

        enviar_ack();
        PGM_count++;
        salvar_modulos_na_flash();
      }
    }

    if (esperando_ack) {
      if (Rx_buffer[FUNCTION] == 'T' && pacote_completo && Rx_buffer[DATA] == ACK) {
        esperando_ack = false;
        enviar_pacote = false;
        toggle_on = false;
      }

      if (Rx_buffer[FUNCTION] == 'D' && pacote_completo && Rx_buffer[DATA] == ACK &&
          (Rx_buffer[2] == modulos[modulo_index].UID0 &&
           Rx_buffer[3] == modulos[modulo_index].UID1 &&
           Rx_buffer[4] == modulos[modulo_index].UID2 &&
           Rx_buffer[5] == modulos[modulo_index].UID3)) {
      }
    }

    if (Rx_buffer[FUNCTION] == 'S' && Rx_buffer[ORIGIN] == 0x02 && pacote_completo) {
      get_status = true;
    }

    if ((TAMANHO_BUFFER - Rx_buffer_index) < UART1_RXFIFO_LIMIT) {
      XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit(
          UART1_HW, XMC_USIC_CH_FIFO_SIZE_8WORDS,
          (TAMANHO_BUFFER - Rx_buffer_index) - 1);
    }
  }
}

// Máquina de estados
void Controle() {
#define RECEIVE 0
#define BROADCAST 1
#define STATUS 2
#define TOGGLE 3
#define DELETE 4
#define TRANSMIT 5
#define DELAY_ENVIO 6
#define LIMPAR 7

  switch (estado) {
  case RECEIVE: {
    pacote_completo = false;
    XMC_GPIO_SetOutputHigh(Bus_Controle_PORT, Bus_Controle_PIN);

    if (enviar_pacote) {
      if (toggle_on) {
        esperando_ack = true;
        estado = TOGGLE;
      }

      if (delete_on) {
        estado = DELETE;
      }

      if (broadcast_on) {

        cadastrado = false;
        estado = BROADCAST;
      }
    } else {
      if (get_status) {

        get_status = false;

        switch (Rx_buffer[DATA]) {
        case 0x00:
          status_reles = 0;
          break;
        case 0x01:
          status_reles = 1;
          break;
        case 0x03:
          status_reles = 2;
          break;
        case 0x07:
          status_reles = 3;
          break;
        case 0x0F:
          status_reles = 4;
          break;
        case 0x1F:
          status_reles = 5;
          break;
        }
      }

      if (PGM_cadastrado[1] || PGM_cadastrado[2] || PGM_cadastrado[3] ||
          PGM_cadastrado[4]) {
        estado = STATUS;
      }
    }

  } break;

  case BROADCAST: {

	montar_pacote(12, PGM_ID, 0, 0, 0, 0, 'A', 0x01, PGM_count,
                  Buffer_TX);

    estado = TRANSMIT;

  } break;

  case STATUS: {

	montar_pacote(12, PGM_ID, modulos[modulo_index].UID0, modulos[modulo_index].UID1,
                modulos[modulo_index].UID2, modulos[modulo_index].UID3, 'S', 0x01,
                modulos[modulo_index].numero, Buffer_TX);
                
    estado = TRANSMIT;

  } break;

  case TOGGLE: {
    if (esperando_ack) {
      switch (ligar_reles) {
      case 0:
      	
      	montar_pacote(12, PGM_ID, modulos[modulo_index].UID0, modulos[modulo_index].UID1,
                modulos[modulo_index].UID2, modulos[modulo_index].UID3, 'T', 0x01,
                0x00, Buffer_TX);

        estado = TRANSMIT;
        break;

      case 1:
        montar_pacote(12, PGM_ID, modulos[modulo_index].UID0, modulos[modulo_index].UID1,
                modulos[modulo_index].UID2, modulos[modulo_index].UID3, 'T', 0x01,
                0x01, Buffer_TX);

        estado = TRANSMIT;
        break;

      case 2:
        montar_pacote(12, PGM_ID, modulos[modulo_index].UID0, modulos[modulo_index].UID1,
                modulos[modulo_index].UID2, modulos[modulo_index].UID3, 'T', 0x01,
                0x03, Buffer_TX);

        estado = TRANSMIT;
        break;

      case 3:
        montar_pacote(12, PGM_ID, modulos[modulo_index].UID0, modulos[modulo_index].UID1,
                modulos[modulo_index].UID2, modulos[modulo_index].UID3, 'T', 0x01,
                0x07, Buffer_TX);

        estado = TRANSMIT;
        break;

      case 4:
        montar_pacote(12, PGM_ID, modulos[modulo_index].UID0, modulos[modulo_index].UID1,
                modulos[modulo_index].UID2, modulos[modulo_index].UID3, 'T', 0x01,
                0x0F, Buffer_TX);

        estado = TRANSMIT;
        break;

      case 5:
        montar_pacote(12, PGM_ID, modulos[modulo_index].UID0, modulos[modulo_index].UID1,
                modulos[modulo_index].UID2, modulos[modulo_index].UID3, 'T', 0x01,
                0x1F, Buffer_TX);

        estado = TRANSMIT;
        break;

      default:
        montar_pacote(12, PGM_ID, modulos[modulo_index].UID0, modulos[modulo_index].UID1,
                modulos[modulo_index].UID2, modulos[modulo_index].UID3, 'T', 0x01,
                0x00, Buffer_TX);

        estado = TRANSMIT;
        break;
      }
    } else {
      enviar_pacote = false;
    }

  } break;

  case DELETE: {

    cadastrado = true;
    get_status = false;

    for (int i = 1; i < 5; i++) {
      PGM_cadastrado[i] = false;
    }

	montar_pacote(12, PGM_BROADCAST_ID, 0, 0, 0, 0, 'D', 0x01, 0,
                  Buffer_TX);

    estado = TRANSMIT;

  } break;

  case TRANSMIT: {

    if (broadcast_on) {

      delay_tx = systick + 500;

    } else {
      delay_tx = systick + 150;
    }

    estado = DELAY_ENVIO;

  } break;

  case DELAY_ENVIO: {

    if (systick >= delay_tx) {
      XMC_GPIO_SetOutputLow(Bus_Controle_PORT, Bus_Controle_PIN);
      for (int i = 0; i < sizeof(Buffer_TX); i++) {

        XMC_UART_CH_Transmit(UART1_HW, Buffer_TX[i]);
      }

      while (!XMC_USIC_CH_TXFIFO_IsEmpty(UART1_HW))
        ;

      pacote_obsoleto = true;
      estado = LIMPAR;
    }

  } break;

  case LIMPAR: {
    if (pacote_obsoleto) {
      for (uint8_t i = 0; i < sizeof(Buffer_TX); i++) {
        Buffer_TX[i] = 0;
      }

      pacote_completo = false;
      pacote_obsoleto = false;
    }

    if (delete_on) {
      for (int i = 0; i < 5; i++) {
        modulos[i].numero = 0;
        modulos[i].UID0 = 0;
        modulos[i].UID1 = 0;
        modulos[i].UID2 = 0;
        modulos[i].UID3 = 0;
      }
      PGM_count = 1;
    }

    estado = RECEIVE;

  } break;
  }
}

void blink_led(uint8_t led_index, uint8_t n) {

  if (led_index >= NUM_LEDS)
    return;

  leds[led_index].blink_target = n * 2; // *2 porque liga/desliga conta 2 vezes
  leds[led_index].blink_count = 0;
  leds[led_index].piscando = true;
}
// Rotina quando timer de debounce gerar evento
void CCU40_0_IRQHandler() {
  esperando_debounce = false;
  XMC_CCU4_SLICE_StopTimer(CCU40_CC40);
  XMC_CCU4_SLICE_ClearEvent(CCU40_CC40, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);

  if (XMC_GPIO_GetInput(PB1_PORT, PB1_PIN) == 0) {
    if (ligar_reles >= 5) {
      ligar_reles = 0;
    } else {
      ligar_reles++;
    }
  }

  if (XMC_GPIO_GetInput(PB4_PORT, PB4_PIN) == 0) {
    if (modulo_index >= 2) {
      modulo_index = 1;
    } else {
      modulo_index++;
    }
  }

  if (XMC_GPIO_GetInput(PB3_PORT, PB3_PIN) == 0) {
    led_cadastro = true;
    cadastrado = false;
    enviar_pacote = true;
    broadcast_on = true;
  }

  if (XMC_GPIO_GetInput(PB2_PORT, PB2_PIN) == 0) {
    led_envio = 1;
    toggle_on = true;
    enviar_pacote = true;
  } else {
    led_envio = 0;
  }
}

void deletar_PGM() {
  XMC_GPIO_SetOutputLow(LED_PB1_PORT, LED_PB1_PIN);
  XMC_FLASH_EraseSector((uint32_t *)FLASH_SECTOR_ADDR);

  delete_on = true;
}

// Rotina para controle dos tempos
void SysTick_Handler(void) {
  if (!esperando_debounce) {
    bool pb1_estado = XMC_GPIO_GetInput(PB1_PORT, PB1_PIN);
    bool pb2_estado = XMC_GPIO_GetInput(PB2_PORT, PB2_PIN);
    bool pb3_estado = XMC_GPIO_GetInput(PB3_PORT, PB3_PIN);
    bool pb4_estado = XMC_GPIO_GetInput(PB4_PORT, PB4_PIN);

    if ((!pb1_estado && pb1_ultimo_estado) ||
        (!pb2_estado && pb2_ultimo_estado) ||
        (!pb3_estado && pb3_ultimo_estado) ||
        (!pb4_estado && pb4_ultimo_estado)) {
      // Detectou borda de descida
      esperando_debounce = true;
      XMC_CCU4_SLICE_StartTimer(CCU40_CC40);
    }

    if (!pb2_estado && !pb2_ultimo_estado) {
      led_envio = 1;
    } else {
      led_envio = 0;
    }

    if (!pb1_estado && !pb1_ultimo_estado) {
      Tempo_3000ms = 3000;
      if (--hold_b1 == 0) {
        delete_on = true;
        enviar_pacote = true;
        deletar_PGM();
      }
    } else if (pb1_estado && !pb1_ultimo_estado) {
      delete_on = false;
      enviar_pacote = false;
      XMC_GPIO_SetOutputHigh(LED_PB1_PORT, LED_PB1_PIN);
    } else {
      hold_b1 = 2500;
    }

    pb1_ultimo_estado = pb1_estado;
    pb2_ultimo_estado = pb2_estado;
    pb3_ultimo_estado = pb3_estado;
    pb4_ultimo_estado = pb4_estado;
  }

  if (--Tempo_200ms == 0) {
    Tempo_200ms = 200;
    acionar_200ms = true;
    for (int i = 0; i < NUM_LEDS; i++) {
      if (leds[i].piscando) {
        if (leds[i].blink_count < leds[i].blink_target) {
          XMC_GPIO_ToggleOutput(leds[i].port, leds[i].pin);
          leds[i].blink_count++;
        } else {
          leds[i].piscando = false;
          XMC_GPIO_SetOutputHigh(leds[i].port,
                                 leds[i].pin); // garante desligado
        }
      }
    }
  }

  if (--Tempo_3000ms == 0) {
    Tempo_3000ms = 3000;
    acionar_3000ms = true;
  }

  if (--Tempo_100ms == 0) {
    Tempo_100ms = 100;
    acionar_100ms = true;
  }

  if (sem_comunicação < TIMEOUT_MAX) {
    sem_comunicação++;
  }

  systick++;
}

// Rotina para controlar leds
void Controle_led() {

  if (acionar_3000ms) {
    blink_led(0, ligar_reles);
    blink_led(2, status_reles);
    blink_led(3, modulo_index);
    acionar_3000ms = false;
  }

  if (led_cadastro) {
    if (acionar_100ms) {
      tentando_cadastrar--;
      XMC_GPIO_ToggleOutput(LED_ST_PORT, LED_ST_PIN);
      acionar_100ms = false;
    }

    if (tentando_cadastrar == 0) {
      led_cadastro = false;
      cadastrado = true;
      broadcast_on = false;
      enviar_pacote = false;
      broadcast_validate = false;
      tentando_cadastrar = 100;
    }
  } else {
    broadcast_on = false;
    XMC_GPIO_SetOutputLow(LED_ST_PORT, LED_ST_PIN);
  }

  if (led_envio) {
    XMC_GPIO_SetOutputLow(LED_PB2_PORT, LED_PB2_PIN);
  } else {
    XMC_GPIO_SetOutputHigh(LED_PB2_PORT, LED_PB2_PIN);
  }

  if (sem_comunicação >= TIMEOUT_MAX) {
    blink_led(2, 10);
  }
}

int main(void) {
  cy_rslt_t result;

  /* Inicializa a placa */
  result = cybsp_init();
  if (result != CY_RSLT_SUCCESS) {
    CY_ASSERT(0);
  }

  SysTick_Config(SystemCoreClock / 1000);

  // Inicia UART
  XMC_UART_CH_Start(UART1_HW);

  // Inverte a saída de dados da UART (nível lógico de TX)
  UART1_HW->SCTR = (UART1_HW->SCTR & ~(0X3 << 6)) | (0X1 << 6);

  NVIC_SetPriority(USIC0_1_IRQn, 3U);
  NVIC_EnableIRQ(USIC0_1_IRQn);

  NVIC_SetPriority(CCU40_0_IRQn, 2U);
  NVIC_EnableIRQ(CCU40_0_IRQn);

  NVIC_SetPriority(CCU40_1_IRQn, 2U);
  NVIC_EnableIRQ(CCU40_1_IRQn);

  XMC_GPIO_SetOutputHigh(Bus_Controle_PORT, Bus_Controle_PIN);

  if ((TAMANHO_BUFFER - Rx_buffer_index) < UART1_RXFIFO_LIMIT) {
    XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit(
        UART1_HW, XMC_USIC_CH_FIFO_SIZE_8WORDS,
        (TAMANHO_BUFFER - Rx_buffer_index) - 1);
  }

  carregar_modulos_da_flash();

  for (;;) {

    Controle_led();
    Controle();
  }
}
