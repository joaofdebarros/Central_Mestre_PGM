#include "XMC1300.h"
#include "cybsp.h"
#include "cy_utils.h"
#include "cycfg_peripherals.h"
#include "cycfg_pins.h"
#include "xmc1_gpio.h"
#include "xmc_gpio.h"
#include "xmc_uart.h"
#include "xmc_usic.h"
#include <stdint.h>
#include <xmc_device.h>
#include <xmc_scu.h>

#define TAMANHO_BUFFER 11
//Buffer de envio e recepção de dados
volatile uint8_t Rx_buffer[TAMANHO_BUFFER];
volatile uint8_t Rx_buffer_index = 0;
uint8_t Buffer_TX[TAMANHO_BUFFER] = {0};

//Flags de envio e recepção de dados
volatile bool recebendo = false;
volatile bool pacote_completo = false;  
volatile bool pacote_obsoleto = false;
volatile bool enviando_pacote = false;
volatile bool cadastrado = true;
volatile bool validated = true;
//Variáveis do pacote
#define start_byte 	0x7E
uint8_t UID0 = 0;
uint8_t UID1 = 0;
uint8_t UID2 = 0;
uint8_t UID3 = 0;
uint8_t Function = 0;
uint8_t data_1 = 0;
uint8_t data_2 = 0;
uint8_t checksum = 0;
#define stop_byte 	0x81
uint8_t PGM_count = 1;

//Variáveis de tempo/delays
#define TIMEOUT_MAX 2000
uint32_t sem_comunicação = 0;
uint8_t Tempo_200ms = 10;
uint16_t Tempo_3000ms = 1000;
uint16_t Tempo_100ms = 100;
uint32_t tentando_cadastrar = 100;
uint8_t piscadas = 0;
volatile uint32_t systick = 0;
volatile uint32_t delay_tx = 0;
volatile bool aguardando_envio = false;
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
//Estado dos botões
#define NUM_LEDS 5
uint8_t LD1 = 0;
uint8_t LD3 = 0;
uint8_t b4 = 1;
uint8_t blink1 = 0;
uint8_t blink2 = 0;
uint8_t blink3 = 0;
uint8_t blink4 = 0;
volatile bool LD2 = 0;
volatile bool led_cadastro = false;
volatile bool PGM_cadastrado[5] = {0,0,0,0,0};

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
}PGM_t;

PGM_t modulos[5] = {
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0},
};

//Máquina de estados
uint8_t estado = 2;

//Rotina para receber dados
void USIC0_1_IRQHandler(void)
{
	if(pacote_completo == false){
		
		while(!XMC_USIC_CH_RXFIFO_IsEmpty(UART1_HW)){
			
			uint8_t rx = XMC_UART_CH_GetReceivedData(UART1_HW);
			if(!recebendo){
				if(rx == 0x7E){
				recebendo = true;
				Rx_buffer_index = 0;
				}
			}else{
				if(rx == 0x81){
					recebendo = false;
					if(Rx_buffer[6] == 0x02)
					{
						pacote_completo = true;
						sem_comunicação = 0;
					}
					
					Rx_buffer_index = 0;
					
					break;
				}else{
					
					if(Rx_buffer_index < TAMANHO_BUFFER){
						Rx_buffer[Rx_buffer_index++] = rx;
					}else{
						recebendo = false;
						Rx_buffer_index = 0;
					}
				}
			}
			
		}
		
		if((Rx_buffer[5] == 'A') && pacote_completo && (Rx_buffer[1] != 0 || Rx_buffer[2] != 0 || Rx_buffer[3] != 0 || Rx_buffer[4] != 0))
		{
			for(int i = 1;  i < 6; i++)
			{
				if(Rx_buffer[1] == modulos[i].UID0 && Rx_buffer[2] == modulos[i].UID1 && Rx_buffer[3] == modulos[i].UID2 && Rx_buffer[4] == modulos[i].UID3)
				{
					validated = true;
					cadastrado = true;
					break;
				}
			}
			
			if(PGM_count < 5 && !cadastrado)
			{
				led_cadastro = false;
				cadastrado = true;
				PGM_cadastrado[PGM_count] = true;
				modulos[PGM_count].numero = PGM_count;
				modulos[PGM_count].UID0 = Rx_buffer[1];
				modulos[PGM_count].UID1 = Rx_buffer[2];
				modulos[PGM_count].UID2 = Rx_buffer[3];
				modulos[PGM_count].UID3 = Rx_buffer[4];
				
				PGM_count++;
			}
		}
		
		if(Rx_buffer[5] == 'S' && Rx_buffer[6] == 0x02){get_status = true;}
		
		if((TAMANHO_BUFFER - Rx_buffer_index) < UART1_RXFIFO_LIMIT)
	    {
	        XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit(UART1_HW, XMC_USIC_CH_FIFO_SIZE_8WORDS, (TAMANHO_BUFFER - Rx_buffer_index) - 1);
	    }
	}
    
}

 //Rotina para enviar pacote
void enviar_pacote_uart()
{
 
    uint8_t pacote[11] = {
			        start_byte,
			        TAMANHO_BUFFER,
					modulos[b4].UID0,
					modulos[b4].UID1,
					modulos[b4].UID2,
					modulos[b4].UID3,
			        Function,
			        data_1,
			        data_2,
			        checksum,
			        stop_byte,
			};
			
	checksum = start_byte ^ TAMANHO_BUFFER ^ UID0 ^ UID1 ^ UID2 ^ UID3 ^ Function ^ data_1 ^ data_2;
    checksum = ~checksum;
    
	switch (LD1) {
		case 0:	
				pacote[1] = TAMANHO_BUFFER;
				pacote[2] = modulos[b4].UID0;
				pacote[3] = modulos[b4].UID1;
				pacote[4] = modulos[b4].UID2;
				pacote[5] = modulos[b4].UID3;
				pacote[6] = 'T';
				pacote[7] = 0x01;
				pacote[8] = 0x00;
				pacote[9] = pacote[0] ^ pacote[1] ^ pacote[2] ^ pacote[3] ^ pacote[4] ^ pacote[5] ^ pacote[6] ^ pacote[7] ^ pacote[8];
    			pacote[9] = ~pacote[9];
				break;
		
		case 1:
				pacote[1] = TAMANHO_BUFFER;
				pacote[2] = modulos[b4].UID0;
				pacote[3] = modulos[b4].UID1;
				pacote[4] = modulos[b4].UID2;
				pacote[5] = modulos[b4].UID3;
				pacote[6] = 'T';
				pacote[7] = 0x01;
				pacote[8] = 0x01;
				pacote[9] = pacote[0] ^ pacote[1] ^ pacote[2] ^ pacote[3] ^ pacote[4] ^ pacote[5] ^ pacote[6] ^ pacote[7] ^ pacote[8];
    			pacote[9] = ~pacote[9];
				break;
		
		case 2:
				pacote[1] = TAMANHO_BUFFER;
				pacote[2] = modulos[b4].UID0;
				pacote[3] = modulos[b4].UID1;
				pacote[4] = modulos[b4].UID2;
				pacote[5] = modulos[b4].UID3;
				pacote[6] = 'T';
				pacote[7] = 0x01;
				pacote[8] = 0x03;
				pacote[9] = pacote[0] ^ pacote[1] ^ pacote[2] ^ pacote[3] ^ pacote[4] ^ pacote[5] ^ pacote[6] ^ pacote[7] ^ pacote[8];
    			pacote[9] = ~pacote[9];
				break;
		
		case 3:
				pacote[1] = TAMANHO_BUFFER;
				pacote[2] = modulos[b4].UID0;
				pacote[3] = modulos[b4].UID1;
				pacote[4] = modulos[b4].UID2;
				pacote[5] = modulos[b4].UID3;
				pacote[6] = 'T';
				pacote[7] = 0x01;
				pacote[8] = 0x07;
				pacote[9] = pacote[0] ^ pacote[1] ^ pacote[2] ^ pacote[3] ^ pacote[4] ^ pacote[5] ^ pacote[6] ^ pacote[7] ^ pacote[8];
    			pacote[9] = ~pacote[9];
				break;
		
		case 4:
				pacote[1] = TAMANHO_BUFFER;
				pacote[2] = modulos[b4].UID0;
				pacote[3] = modulos[b4].UID1;
				pacote[4] = modulos[b4].UID2;
				pacote[5] = modulos[b4].UID3;
				pacote[6] = 'T';
				pacote[7] = 0x01;
				pacote[8] = 0x0F;
				pacote[9] = pacote[0] ^ pacote[1] ^ pacote[2] ^ pacote[3] ^ pacote[4] ^ pacote[5] ^ pacote[6] ^ pacote[7] ^ pacote[8];
    			pacote[9] = ~pacote[9];
				break;
		
		case 5:
				pacote[1] = TAMANHO_BUFFER;
				pacote[2] = modulos[b4].UID0;
				pacote[3] = modulos[b4].UID1;
				pacote[4] = modulos[b4].UID2;
				pacote[5] = modulos[b4].UID3;
				pacote[6] = 'T';
				pacote[7] = 0x01;
				pacote[8] = 0x1F;
				pacote[9] = pacote[0] ^ pacote[1] ^ pacote[2] ^ pacote[3] ^ pacote[4] ^ pacote[5] ^ pacote[6] ^ pacote[7] ^ pacote[8];
    			pacote[9] = ~pacote[9];
				break;
				
		default:
				pacote[1] = TAMANHO_BUFFER;
				pacote[2] = modulos[b4].UID0;
				pacote[3] = modulos[b4].UID1;
				pacote[4] = modulos[b4].UID2;
				pacote[5] = modulos[b4].UID3;
				pacote[6] = 'S';
				pacote[7] = 0x01;
				pacote[8] = 0x00;
				pacote[9] = pacote[0] ^ pacote[1] ^ pacote[2] ^ pacote[3] ^ pacote[4] ^ pacote[5] ^ pacote[6] ^ pacote[7] ^ pacote[8];
    			pacote[9] = ~pacote[9];
				break;
	}
    // Monta pacote final
    

    // Envia byte a byte
    XMC_GPIO_SetOutputLow(Bus_Controle_PORT, Bus_Controle_PIN);
    for (int i = 0; i < sizeof(pacote); i++)
    {
        XMC_UART_CH_Transmit(UART1_HW, pacote[i]);
    }
    
    while(!XMC_USIC_CH_TXFIFO_IsEmpty(UART1_HW));
    
    XMC_GPIO_SetOutputHigh(Bus_Controle_PORT, Bus_Controle_PIN);
    
    for(uint8_t i=0; i < sizeof(pacote); i++){
		pacote[i] = 0;
	}
	
}

//Máquina de estados
void Controle()
{
	#define RECEIVE		0
	#define CADASTRO	1
	#define STATUS		2
	#define TRANSMIT	3
	#define DELAY_ENVIO	4
	#define LIMPAR		5
	
	switch (estado) 
	{
		case RECEIVE:
		{
			pacote_completo = false;
			XMC_GPIO_SetOutputHigh(Bus_Controle_PORT, Bus_Controle_PIN);
			
			if(!cadastrado)
			{
				validated = false;
				estado = CADASTRO;
			}else if(cadastrado && !validated){
				
				Buffer_TX[0] = start_byte;
				Buffer_TX[1] = TAMANHO_BUFFER;
				Buffer_TX[2] = modulos[b4].UID0;
				Buffer_TX[3] = modulos[b4].UID1;
				Buffer_TX[4] = modulos[b4].UID2;
				Buffer_TX[5] = modulos[b4].UID3;
				Buffer_TX[6] = 'A';
				Buffer_TX[7] = 0x01;
				Buffer_TX[8] = (PGM_count - 1);
				Buffer_TX[9] = ~(Buffer_TX[0] ^ Buffer_TX[1] ^ Buffer_TX[2] ^ Buffer_TX[3] ^ Buffer_TX[4] ^ Buffer_TX[5] ^ Buffer_TX[6] ^ Buffer_TX[7] ^ Buffer_TX[8]);
		    	Buffer_TX[10] = stop_byte;
			    	
			    estado = TRANSMIT;
			}else if(cadastrado && validated){
				if(get_status){
				
					get_status = false;
					
					switch (Rx_buffer[7]) 
					{
						case 0x00:	
									LD3 = 0;
									break;				
						case 0x01:	
									LD3 = 1;
									break;		
						case 0x03:	
									LD3 = 2;
									break;		
						case 0x07:	
									LD3 = 3;
									break;	
						case 0x0F:	
									LD3 = 4;
									break;			
						case 0x1F:	
									LD3 = 5;
									break;	
					}	
			 	}

				if(PGM_cadastrado[1] || PGM_cadastrado[2] || PGM_cadastrado[3] || PGM_cadastrado[4]){
					estado = STATUS;
				}
			}else{
				estado = RECEIVE;
			}
			
		}break;
		
		case CADASTRO:
		{
			Buffer_TX[0] = start_byte;
			Buffer_TX[1] = TAMANHO_BUFFER;
			Buffer_TX[2] = 0x00;
			Buffer_TX[3] = 0x00;
			Buffer_TX[4] = 0x00;
			Buffer_TX[5] = 0x00;
			Buffer_TX[6] = 'A';
			Buffer_TX[7] = 0x01;
			Buffer_TX[8] = PGM_count;
			Buffer_TX[9] = ~(Buffer_TX[0] ^ Buffer_TX[1] ^ Buffer_TX[2] ^ Buffer_TX[3] ^ Buffer_TX[4] ^ Buffer_TX[5] ^ Buffer_TX[6] ^ Buffer_TX[7] ^ Buffer_TX[8]);
	    	Buffer_TX[10] = stop_byte;
		    	
		    estado = TRANSMIT;
			
			
		}break;
		
		case STATUS:
		{
			
			Buffer_TX[0] = start_byte;
			Buffer_TX[1] = TAMANHO_BUFFER;
			Buffer_TX[2] = modulos[b4].UID0;
			Buffer_TX[3] = modulos[b4].UID1;
			Buffer_TX[4] = modulos[b4].UID2;
			Buffer_TX[5] = modulos[b4].UID3;
			Buffer_TX[6] = 'S';
			Buffer_TX[7] = 0x01;
			Buffer_TX[8] = modulos[b4].numero;
			Buffer_TX[9] = ~(Buffer_TX[0] ^ Buffer_TX[1] ^ Buffer_TX[2] ^ Buffer_TX[3] ^ Buffer_TX[4] ^ Buffer_TX[5] ^ Buffer_TX[6] ^ Buffer_TX[7] ^ Buffer_TX[8]);
		    Buffer_TX[10] = stop_byte;
		    estado = TRANSMIT;
			
		}break;
		
		case TRANSMIT:
		{
			
			delay_tx = systick + 200;
			aguardando_envio = true;		    
		    estado = DELAY_ENVIO;
			
		}break;
		
		case DELAY_ENVIO:
		{
			if(systick >= delay_tx)
			{
				
				
				XMC_GPIO_SetOutputLow(Bus_Controle_PORT, Bus_Controle_PIN);
				for (int i = 0; i < sizeof(Buffer_TX); i++)
			    {
					
			        XMC_UART_CH_Transmit(UART1_HW, Buffer_TX[i]);
			        
			    }
			    
			    while(!XMC_USIC_CH_TXFIFO_IsEmpty(UART1_HW));
			    
			    pacote_obsoleto = true;
			    aguardando_envio = false;
			    estado = LIMPAR;
			}
			
			
		}break;
		
		case LIMPAR:
		{
			if(pacote_obsoleto)
			{
				for(uint8_t i=0; i<sizeof(Buffer_TX); i++){
					Buffer_TX[i] = 0;
				}
				pacote_completo = false;
				pacote_obsoleto = false;
				
			}
			
			estado = RECEIVE;
			
		}break;
		
	}
}

//Rotina quando timer de debounce gerar evento
void CCU40_0_IRQHandler()
{
		esperando_debounce = false;
		XMC_CCU4_SLICE_StopTimer(CCU40_CC40);
    	XMC_CCU4_SLICE_ClearEvent(CCU40_CC40, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);

		if(XMC_GPIO_GetInput(PB1_PORT, PB1_PIN) == 0){
			if(LD1 >= 5){
				LD1 = 0;
			}else{
				LD1++;
			}
		}
		
		if(XMC_GPIO_GetInput(PB4_PORT, PB4_PIN) == 0){
			if(b4 >= 2){
				b4 = 1;
			}else{
				b4++;
			}
		}
		
		if(XMC_GPIO_GetInput(PB3_PORT, PB3_PIN) == 0){
			led_cadastro = true;
			cadastrado = false;

		}
			
		if(XMC_GPIO_GetInput(PB2_PORT, PB2_PIN) == 0){
			LD2 = 1;
			enviar_pacote_uart();
		}else{
			LD2 = 0;
		}
		
}

//Rotina para controle dos tempos
void SysTick_Handler(void)
{
	if (!esperando_debounce)
    {
        bool pb1_estado = XMC_GPIO_GetInput(PB1_PORT, PB1_PIN);
        bool pb2_estado = XMC_GPIO_GetInput(PB2_PORT, PB2_PIN);
        bool pb3_estado = XMC_GPIO_GetInput(PB3_PORT, PB3_PIN);
        bool pb4_estado = XMC_GPIO_GetInput(PB4_PORT, PB4_PIN);

        if ((!pb1_estado && pb1_ultimo_estado) || (!pb2_estado && pb2_ultimo_estado) || (!pb3_estado && pb3_ultimo_estado) || (!pb4_estado && pb4_ultimo_estado))
        {
            // Detectou borda de descida
            esperando_debounce = true;
            XMC_CCU4_SLICE_StartTimer(CCU40_CC40);
        }else if(!pb2_estado && !pb2_ultimo_estado){
			LD2 = 1;
		}else{
			LD2 = 0;
		}
        

        pb1_ultimo_estado = pb1_estado;
        pb2_ultimo_estado = pb2_estado;
        pb3_ultimo_estado = pb3_estado;
        pb4_ultimo_estado = pb4_estado;
    }
	
	if(--Tempo_200ms == 0){
		Tempo_200ms = 200;
		acionar_200ms = true;
		for(int i=0; i<NUM_LEDS; i++){
			if (leds[i].piscando) {
	            if (leds[i].blink_count < leds[i].blink_target) {
	                XMC_GPIO_ToggleOutput(leds[i].port, leds[i].pin);
	                leds[i].blink_count++;
	            } else {
	                leds[i].piscando = false;
	                XMC_GPIO_SetOutputHigh(leds[i].port, leds[i].pin); // garante desligado
	            }
	        }
		}
	}
		
	if(--Tempo_3000ms == 0){
		Tempo_3000ms = 3000;
		acionar_3000ms = true;	
	}
	
	if(--Tempo_100ms == 0){
		Tempo_100ms = 100;
		acionar_100ms = true;			
	}
	
	if(sem_comunicação < TIMEOUT_MAX)
	{
		sem_comunicação++;
	}
	
	systick++;
}


void blink_led(uint8_t led_index, uint8_t n)
{
	
	if (led_index >= NUM_LEDS) return;

    leds[led_index].blink_target = n * 2; // *2 porque liga/desliga conta 2 vezes
    leds[led_index].blink_count = 0;
    leds[led_index].piscando = true;

}
//Rotina para controlar leds
void Controle_led(){
	
	if(acionar_3000ms){
		blink_led(0, LD1);
		blink_led(2, LD3);
		blink_led(3, b4);
		acionar_3000ms = false;
	}
	
	if(led_cadastro)
	{
		if(acionar_100ms)
		{
			tentando_cadastrar--;
			XMC_GPIO_ToggleOutput(LED_ST_PORT, LED_ST_PIN);
			acionar_100ms = false;
		}
		
		if(tentando_cadastrar == 0)
		{
			validated = true;
			cadastrado = true;
			led_cadastro = false;
			tentando_cadastrar = 100;
		}
	}else{
		XMC_GPIO_SetOutputLow(LED_ST_PORT, LED_ST_PIN);
	}
	
	
	if(LD2){
		XMC_GPIO_SetOutputLow(LED_PB2_PORT, LED_PB2_PIN);
	}else{
		XMC_GPIO_SetOutputHigh(LED_PB2_PORT, LED_PB2_PIN);
	}
	
	if(sem_comunicação >= TIMEOUT_MAX)
	{
	    blink_led(2, 10);
	}
	
}

int main(void)
{
    cy_rslt_t result;

    /* Inicializa a placa */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
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
    
    
    if((TAMANHO_BUFFER - Rx_buffer_index) < UART1_RXFIFO_LIMIT)
    {
        XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit(UART1_HW, XMC_USIC_CH_FIFO_SIZE_8WORDS, (TAMANHO_BUFFER - Rx_buffer_index) - 1);
    }
    
    
    for (;;)
    {
		Controle_led();
		Controle();
    }
}


