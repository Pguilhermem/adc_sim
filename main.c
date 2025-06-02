//#############################################################################
//
// ARQUIVO:    ex2_multi_channel_adc.c
//
// TÍTULO:    Amostragem e Filtragem de Múltiplos Canais ADC
//
//! Este exemplo simula aquisição de dados ADC de múltiplos canais.
//! Ele utiliza STRUCTS para organizar os dados de cada canal,
//! PONTEIROS para acessar esses structs, e ENUMERAÇÕES para definir
//! o estado de cada canal. A filtragem de média móvel é aplicada.
//! Observar os dados e estados no depurador do CCS.
//
//#############################################################################
//
// $Data de Lançamento: $
// $Copyright:
// Copyright (C) 2013-2023 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribuição e uso em formatos de código-fonte e binários, com ou sem
// modificação, são permitidos desde que as seguintes condições sejam
// atendidas:
//
//   As redistribuições do código-source devem reter o aviso de direitos autorais
//   acima, esta lista de condições e a seguinte isenção de responsabilidade.
//
//   As redistribuições em formato binário devem reproduzir o aviso de direitos autorais
//   acima, esta lista de condições e a seguinte isenção de responsabilidade na
//   documentação e/ou outros materiais fornecidos com a distribuição.
//
//   Nem o nome da Texas Instruments Incorporated nem os nomes de
//   seus colaboradores podem ser usados para endossar ou promover produtos derivados
//   do software sem permissão prévia por escrito.
//
// ESTE SOFTWARE É FORNECIDO PELOS DETENTORES DOS DIREITOS AUTORAIS E COLABORADORES
// "AS IS" E QUAISQUER GARANTIAS EXPRESSAS OU IMPLÍCITAS, INCLUINDO, MAS NÃO
// SE LIMITANDO A, AS GARANTIAS IMPLÍCITAS DE COMERCIALIZAÇÃO E ADEQUAÇÃO PARA
// UM PROPÓSITO ESPECÍFICO SÃO REJEITADAS. EM NENHUM CASO O DETENTOR DOS DIREITOS AUTORAIS
// OU COLABORADORES SERÃO RESPONSÁVEIS POR QUAISQUER DANOS DIRETOS, INDIRETOS, INCIDENTALMENTE,
// ESPECIAIS, EXEMPLARES OU CONSEQUENCIAIS (INCLUINDO, MAS NÃO SE LIMITANDO A,
// AQUISIÇÃO DE BENS OU SERVIÇOS SUBSTITUTOS; PERDA DE USO, DADOS OU LUCROS;
// OU INTERRUPÇÃO DE NEGÓCIOS) SEJA QUAL FOR A CAUSA E SOB QUALQUER TEORIA DE
// RESPONSABILIDADE, SEJA EM CONTRATO, RESPONSABILIDADE ESTRITA OU ATO ILÍCITO
// (INCLUINDO NEGLIGÊNCIA OU OUTRO) DECORRENTE DE QUALQUER FORMA DO USO DESTE
// SOFTWARE, MESMO SE AVISADO DA POSSIBILIDADE DE TAL DANO.
// $
//#############################################################################

// Arquivos Incluídos
#include "driverlib.h"
#include "device.h"
#include <stdbool.h> // Para tipo bool
#include <stdlib.h>  // Para rand()
#include <math.h>    // Para sinf() e M_PI_F (se definido)

// --- Definições Globais ---
#define ADC_MAX_VALUE           4095U   // Valor máximo para ADC de 12 bits
#define ADC_VOLTAGE_REF         3.3F    // Tensão de referência do ADC

#define FILTER_BUFFER_SIZE      16      // Amostras para o filtro de média móvel por canal
#define HISTORY_BUFFER_SIZE     200     // Pontos para histórico por canal (2 ciclos de 1Hz @ 100Hz)
#define NUM_ADC_CHANNELS        3       // Número de canais ADC simulados

#define SINE_AMPLITUDE_ADC      1000.0F // Amplitude da senoide (com ruído)
#define SINE_OFFSET_ADC         2048.0F // Offset da senoide

#define BASE_SINE_FREQUENCY_HZ  1.0F    // Frequência base da senoide (Canal 0)
#define SAMPLING_PERIOD_US      10000U  // Período de amostragem (10ms -> 100Hz)
#define SAMPLING_RATE_HZ        (1000000.0F / SAMPLING_PERIOD_US) // Taxa de amostragem

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846F
#endif

// --- Enumeração para o Estado do Canal ADC ---
typedef enum {
    ADC_CHANNEL_STATE_DISABLED,
    ADC_CHANNEL_STATE_ENABLED,
    ADC_CHANNEL_STATE_OVERRANGE, // Exemplo de estado de erro: valor acima do limite
    ADC_CHANNEL_STATE_UNDERRANGE // Exemplo de estado de erro: valor abaixo do limite
} AdcChannelState_t;

// --- Estrutura (Struct) para um Canal ADC ---
typedef struct {
    unsigned int      buffer[FILTER_BUFFER_SIZE];    // Buffer circular de amostras
    unsigned int      currentIndex;                  // Índice atual do buffer
    unsigned int      filteredValueADC;              // Valor filtrado em contagens ADC
    float             filteredVoltage;               // Valor filtrado em Volts
    AdcChannelState_t state;                         // Estado atual do canal
    unsigned int      channelID;                     // Identificador do canal (0, 1, 2...)
    // Variáveis para geração de sinal simulado (por canal)
    unsigned long     sineSampleCounter;
    float             sineFrequencyHz;
} AdcChannel_t;

// --- Variáveis Globais ---
AdcChannel_t g_adcChannels[NUM_ADC_CHANNELS]; // Array de structs para múltiplos canais

// --- Arrays para Histórico (Observar no depurador) ---
// Estes são arrays 2D para armazenar o historic de cada canal
unsigned int g_rawValuesHistory[NUM_ADC_CHANNELS][HISTORY_BUFFER_SIZE];
unsigned int g_filteredValuesHistory[NUM_ADC_CHANNELS][HISTORY_BUFFER_SIZE];
unsigned int g_historyIndex = 0U; // Índice global para o buffer de histórico (circular)

// Protótipos de Funções
void initAdcChannels(void);
void processAdcChannel(AdcChannel_t *pChannel); // Recebe ponteiro para o canal
unsigned int readSimulatedADC(AdcChannel_t *pChannel);
void addSampleToBuffer(AdcChannel_t *pChannel, unsigned int newSample);
void calculateMovingAverage(AdcChannel_t *pChannel);
float convertADCToVoltage(unsigned int adcValue);
// storeHistory agora apenas escreve, o avanço do índice global é no main()
void storeHistory(unsigned int channelID, unsigned int rawValue, unsigned int filteredValue);

// Função Principal
void main(void)
{
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    EINT; // Habilita Interrupções Globais
    ERTM; // Habilita Depuração em Tempo Real

    initAdcChannels(); // Inicializa todos os canais ADC

    // Loop principal de simulação e filtragem
    for(;;)
    {
        for (unsigned int i = 0U; i < NUM_ADC_CHANNELS; i++) // Processa cada canal ADC
        {
            if (g_adcChannels[i].state != ADC_CHANNEL_STATE_DISABLED) // Se canal estiver habilitado
            {
                processAdcChannel(&g_adcChannels[i]); // Passa o endereço do struct do canal
            }
        }

        // --- AVANÇA O ÍNDICE DO HISTÓRICO GLOBAL APENAS UMA VEZ POR CICLO DE AMOSTRAGEM ---
        g_historyIndex = (g_historyIndex + 1U) % HISTORY_BUFFER_SIZE;

        // Atraso para simular o tempo de amostragem do sistema (para todos os canais)
        DEVICE_DELAY_US(SAMPLING_PERIOD_US);

    }
}

// Implementações de Funções

// Inicializa todos os canais ADC e seus buffers.
void initAdcChannels(void)
{
    for (unsigned int i = 0U; i < NUM_ADC_CHANNELS; i++)
    {
        AdcChannel_t *pCh = &g_adcChannels[i]; // Ponteiro para o struct do canal

        // Inicializa buffer do filtro do canal
        for (unsigned int j = 0U; j < FILTER_BUFFER_SIZE; j++)
        {
            pCh->buffer[j] = 0U;
        }
        pCh->currentIndex = 0U;
        pCh->filteredValueADC = 0U;
        pCh->filteredVoltage = 0.0F;
        pCh->state = ADC_CHANNEL_STATE_ENABLED; // Habilita o canal por padrão
        pCh->channelID = i;
        pCh->sineSampleCounter = 0UL;
        // Frequências diferentes para cada canal para visualização distinta
        pCh->sineFrequencyHz = BASE_SINE_FREQUENCY_HZ * (1.0F + (float)i * 0.5F);
    }

    // Inicializa buffers de histórico (2D)
    for(unsigned int i = 0U; i < NUM_ADC_CHANNELS; i++) {
        for(unsigned int j = 0U; j < HISTORY_BUFFER_SIZE; j++) {
            g_rawValuesHistory[i][j] = 0U;
            g_filteredValuesHistory[i][j] = 0U;
        }
    }
    g_historyIndex = 0U; // Zera o índice global de histórico
}

// Processa um único canal ADC: lê, filtra e armazena no histórico.
void processAdcChannel(AdcChannel_t *pChannel)
{
    unsigned int simulatedADC = readSimulatedADC(pChannel);
    addSampleToBuffer(pChannel, simulatedADC);
    calculateMovingAverage(pChannel);
    pChannel->filteredVoltage = convertADCToVoltage(pChannel->filteredValueADC);

    // Verifica over/under range para atualizar o estado do canal
    if (simulatedADC > ADC_MAX_VALUE) { // Sinal bruto estourou
        pChannel->state = ADC_CHANNEL_STATE_OVERRANGE;
    } else if (simulatedADC < 0U) { // Sinal bruto abaixo de zero (improvavel para unsigned, mas boa checagem)
        pChannel->state = ADC_CHANNEL_STATE_UNDERRANGE;
    } else { // Dentro da faixa normal
        pChannel->state = ADC_CHANNEL_STATE_ENABLED;
    }

    // Armazena a amostra no histórico do canal correspondente
    storeHistory(pChannel->channelID, simulatedADC, pChannel->filteredValueADC);
}

// Simula a leitura ADC de um canal específico, gerando senoide com ruído.
unsigned int readSimulatedADC(AdcChannel_t *pChannel)
{
    float currentSineValue = SINE_OFFSET_ADC +
                            SINE_AMPLITUDE_ADC * sinf(2.0F * M_PI_F * pChannel->sineFrequencyHz * (pChannel->sineSampleCounter / SAMPLING_RATE_HZ));

    float currentNoiseValue = (float)((rand() % 100) - 50); // Ruído de -50 a +49

    float rawADCValueFloat = currentSineValue + currentNoiseValue;

    // Garante que o valor esteja dentro dos limites do ADC
    if (rawADCValueFloat > (float)ADC_MAX_VALUE) rawADCValueFloat = (float)ADC_MAX_VALUE;
    if (rawADCValueFloat < 0.0F) rawADCValueFloat = 0.0F;

    pChannel->sineSampleCounter++; // Avança o contador de amostras do canal

    return (unsigned int)rawADCValueFloat;
}

// Adiciona nova amostra ao buffer circular do canal específico.
void addSampleToBuffer(AdcChannel_t *pChannel, unsigned int newSample)
{
    pChannel->buffer[pChannel->currentIndex] = newSample;
    pChannel->currentIndex = (pChannel->currentIndex + 1U) % FILTER_BUFFER_SIZE;
}

// Calcula a média móvel das amostras no buffer de um canal específico.
void calculateMovingAverage(AdcChannel_t *pChannel)
{
    unsigned long sum = 0UL;
    for (unsigned int i = 0U; i < FILTER_BUFFER_SIZE; i++)
    {
        sum = sum + pChannel->buffer[i];
    }
    pChannel->filteredValueADC = (unsigned int)((float)sum / FILTER_BUFFER_SIZE + 0.5F);

    // Garante que o valor filtrado esteja dentro dos limites do ADC
    if (pChannel->filteredValueADC > ADC_MAX_VALUE) pChannel->filteredValueADC = ADC_MAX_VALUE;
}

// Converte valor ADC (unsigned int) para tensão (Volts).
float convertADCToVoltage(unsigned int adcValue)
{
    return ((float)adcValue / ADC_MAX_VALUE) * ADC_VOLTAGE_REF;
}

// Armazena os valores brutos e filtrados nos arrays de histórico para o canal especificado.
// g_historyIndex é avançado UMA VEZ no loop principal.
void storeHistory(unsigned int channelID, unsigned int rawValue, unsigned int filteredValue)
{
    if (channelID < NUM_ADC_CHANNELS) // Garante que o ID do canal é válido
    {
        g_rawValuesHistory[channelID][g_historyIndex] = rawValue;
        g_filteredValuesHistory[channelID][g_historyIndex] = filteredValue;
    }
}
