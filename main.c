//#############################################################################
//
// ARQUIVO:    adc_sim.c
//
// TÍTULO:    Amostragem e Filtragem de um Canal ADC
//
//! Este exemplo simula a aquisição de um único canal ADC.
//! Utiliza STRUCT para organizar os dados do canal,
//! PONTEIRO para acessar o struct, e ENUMERAÇÃO para definir
//! o estado do canal (NORMAL ou ALERTA). A filtragem é feita por
//! média móvel simples. NÃO utiliza arrays de histórico.
//! Observar os dados e o estado no depurador do CCS.
//
//#############################################################################
//
// $Data de Lançamento: $
// $Copyright:
// Copyright (C) 2013-2023 Texas Instruments Incorporated - http://www.ti.com/
//
// (aviso de copyright original mantido, omitido aqui por brevidade)
// $
//#############################################################################

// Arquivos Incluídos
#include "driverlib.h"
#include "device.h"
#include <stdbool.h> // Para tipo bool

// --- Definições Globais ---
#define ADC_MAX_VALUE           4095U   // Valor máximo para ADC de 12 bits
#define ADC_REFERENCE_VOLTAGE   3.3F    // Tensão de referência do ADC

#define FILTER_BUFFER_SIZE      16      // Número de amostras para a média móvel

#define SAMPLING_PERIOD_US      10000U  // Período de amostragem (10 ms → 100 Hz)

// --- Enumeração para o Estado do Canal ADC ---
typedef enum {
    ADC_CHANNEL_STATE_DISABLED,
    ADC_CHANNEL_STATE_NORMAL,
    ADC_CHANNEL_STATE_ALERT      // Valor acima do limiar de segurança
} AdcChannelState_t;

// --- Estrutura (Struct) para o Canal ADC ---
typedef struct {
    unsigned int      buffer[FILTER_BUFFER_SIZE]; // Buffer circular
    unsigned int      currentIndex;               // Índice atual no buffer
    unsigned int      filteredValueADC;           // Valor filtrado (contagens ADC)
    float             filteredVoltage;             // Valor filtrado em Volts
    AdcChannelState_t state;                      // Estado atual do canal
} AdcChannel_t;

// --- Variável Global Única (apenas um canal) ---
AdcChannel_t g_adcChannel;

// Protótipos de Funções
void initAdcChannel(void);
void processAdcChannel(AdcChannel_t *pChannel);
unsigned int readSimulatedADC(void);
void addSampleToBuffer(AdcChannel_t *pChannel, unsigned int newSample);
void calculateMovingAverage(AdcChannel_t *pChannel);
float convertADCToVoltage(unsigned int adcValue);

// Função Principal
void main(void)
{
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    EINT;
    ERTM;

    initAdcChannel();

    // Loop principal de amostragem e filtragem
    for(;;)
    {
        processAdcChannel(&g_adcChannel);

        // Atraso para simular período de amostragem
        DEVICE_DELAY_US(SAMPLING_PERIOD_US);
    }
}

// Implementações de Funções

void initAdcChannel(void)
{
    AdcChannel_t *pCh = &g_adcChannel;

    // Limpa o buffer do filtro
    for (unsigned int i = 0U; i < FILTER_BUFFER_SIZE; i++)
    {
        pCh->buffer[i] = 0U;
    }
    pCh->currentIndex = 0U;
    pCh->filteredValueADC = 0U;
    pCh->filteredVoltage = 0.0F;
    pCh->state = ADC_CHANNEL_STATE_NORMAL;
}

// Processa um ciclo completo: leitura, filtro e verificação de limiar
void processAdcChannel(AdcChannel_t *pChannel)
{
    // 1. Lê o ADC (simulado)
    unsigned int rawSample = readSimulatedADC();

    // 2. Adiciona a nova amostra ao buffer circular
    addSampleToBuffer(pChannel, rawSample);

    // 3. Calcula a média móvel
    calculateMovingAverage(pChannel);

    // 4. Converte o valor filtrado para tensão
    pChannel->filteredVoltage = convertADCToVoltage(pChannel->filteredValueADC);

    // 5. Verifica o estado do canal com base em um limiar de segurança
    //    (Limiar simples: 80% do fundo de escala, ou seja, ≈ 2,64 V)
    if (pChannel->filteredVoltage > 2.64F)
    {
        pChannel->state = ADC_CHANNEL_STATE_ALERT;
    }
    else
    {
        pChannel->state = ADC_CHANNEL_STATE_NORMAL;
    }
}

// Simula uma leitura do ADC gerando um valor que varia lentamente.
// Utiliza um contador simples para produzir uma rampa triangular
// (sobe de 0 até ADC_MAX_VALUE e depois desce).
unsigned int readSimulatedADC(void)
{
    static unsigned long counter = 0UL;
    static int direction = 1;           // 1 = subindo, -1 = descendo

    // Atualiza o contador com passo fixo
    counter = counter + (unsigned long)(10 * direction);

    // Inverte a direção nos extremos
    if (counter >= ADC_MAX_VALUE)
    {
        counter = ADC_MAX_VALUE;
        direction = -1;
    }
    else if (counter == 0UL)
    {
        direction = 1;
    }

    return (unsigned int)counter;
}

// Adiciona uma nova amostra ao buffer circular do canal.
void addSampleToBuffer(AdcChannel_t *pChannel, unsigned int newSample)
{
    pChannel->buffer[pChannel->currentIndex] = newSample;
    pChannel->currentIndex = (pChannel->currentIndex + 1U) % FILTER_BUFFER_SIZE;
}

// Calcula a média móvel das amostras no buffer.
void calculateMovingAverage(AdcChannel_t *pChannel)
{
    unsigned long sum = 0UL;
    for (unsigned int i = 0U; i < FILTER_BUFFER_SIZE; i++)
    {
        sum = sum + pChannel->buffer[i];
    }
    // Arredondamento simples
    pChannel->filteredValueADC = (unsigned int)((sum + FILTER_BUFFER_SIZE / 2U) / FILTER_BUFFER_SIZE);

    // Garante que o valor não ultrapasse o máximo do ADC
    if (pChannel->filteredValueADC > ADC_MAX_VALUE)
    {
        pChannel->filteredValueADC = ADC_MAX_VALUE;
    }
}

// Converte um valor ADC (0 a 4095) para tensão (0 a 3.3 V).
float convertADCToVoltage(unsigned int adcValue)
{
    return ((float)adcValue / (float)ADC_MAX_VALUE) * ADC_REFERENCE_VOLTAGE;
}
