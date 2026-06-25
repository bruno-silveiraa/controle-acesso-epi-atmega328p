/*

PROJETO: CONTROLE DE ACESSO COM ANALISE DE EPI

Resumo da maquina de estados:

ESTADO1 - ACESSO BLOQUEADO
- LED vermelho ligado.
- Aguarda inicio da analise.
- A analise pode iniciar por BOTAO1 ou comando UART '1'.

ESTADO2 - FUNCIONARIO EM ANALISE
- LED amarelo ligado.
- Aguarda aprovacao ou reprovacao.
- Aprovacao por BOTAO2 ou comando UART '2'.
- Reprovacao por BOTAO3 ou comando UART '3'.

ESTADO3 - ACESSO LIBERADO
- LED verde ligado.
- Permanece liberado por 20 segundos.
- Depois retorna automaticamente para ESTADO1.

Comandos UART no monitor serial:
'1' -> Inicia analise.
'2' -> Aprova EPI.
'3' -> Reprova EPI.
's' ou 'S' -> Envia o status atual.

Configuracao do Monitor Serial:
- Baud rate: 9600 bps.

*/


// Define os nomes dos registradores do ATmega328P.
// Exemplos: DDRD, PORTD, PIND, TCCR0A, UCSR0B, UDR0.
#include <avr/io.h>

// Define as macros e vetores usados em interrupcoes.
// Exemplos: sei(), ISR(), TIMER0_COMPA_vect, INT1_vect.
#include <avr/interrupt.h>

/*
CONFIGURACOES GERAIS DO CLOCK E DA UART
*/

// Frequencia do clock do Arduino Uno / ATmega328P.
// O valor de 16 MHz e usado para calcular o baud rate da UART.
#define F_CPU 16000000UL

// Velocidade da comunicacao serial.
// O monitor serial do Tinkercad/Arduino IDE deve ser configurado em 9600.
#define BAUD 9600UL

// Valor carregado nos registradores UBRR0H e UBRR0L.
// Formula do modo assicrono normal da USART:
// UBRR = (F_CPU / (16 * BAUD)) - 1
// Para 16 MHz e 9600 bps:
// UBRR = (16000000 / (16 * 9600)) - 1 = 103,16 aprox. 103
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

/*

Cada definicao abaixo cria uma mascara de bits.
Exemplo:
(1 << PD5) desloca o bit 1 ate a posicao PD5.
Assim, LED_VERMELHO representa apenas o bit do pino PD5.

Isso permite ligar/desligar pinos sem alterar os outros bits do PORTD.
*/

#define LED_VERMELHO (1 << PD5) // Pino digital 5 do Arduino / PD5.
#define LED_AMARELO  (1 << PD6) // Pino digital 6 do Arduino / PD6.
#define LED_VERDE    (1 << PD7) // Pino digital 7 do Arduino / PD7.

/*

Os botoes usam pull-up interno.
Com pull-up:
- Botao solto: pino le nivel logico 1.
- Botao pressionado: pino e ligado ao GND e le nivel logico 0.

Por isso, no codigo, um botao pressionado e testado com:
if(!(PIND & BOTAOx))
*/

#define BOTAO1 (1 << PD3) // Pino digital 3 / PD3 / interrupcao externa INT1.
#define BOTAO2 (1 << PD2) // Pino digital 2 / PD2 / aprova EPI.
#define BOTAO3 (1 << PD4) // Pino digital 4 / PD4 / reprova EPI.

/*

ESTADOS DA MAQUINA

*/

#define ESTADO1 1 // Acesso bloqueado.
#define ESTADO2 2 // Funcionario em analise.
#define ESTADO3 3 // Acesso liberado.

/*

CONSTANTES DE TEMPO

*/

// Tempo em que o acesso fica liberado no ESTADO3.
// 20000 ms = 20 segundos.
#define TEMPO_LIBERADO_MS 20000UL

// Tempo minimo entre duas leituras validas do BOTAO1.
// Isso reduz o efeito de bouncing mecanico do botao.
#define DEBOUNCE_MS 200UL

/*

VARIAVEIS GLOBAIS

*/

// Contador global de tempo.
// Ele e incrementado pela interrupcao do Timer0 a cada 1 ms.
//
// volatile e necessario porque a variavel e alterada dentro de uma ISR.
// Sem volatile, o compilador poderia otimizar a leitura de forma incorreta.
volatile unsigned long tempo_ms = 0;

// Guarda o ultimo instante em que o BOTAO1 foi aceito pela interrupcao.
volatile unsigned long ultimo_botao1_ms = 0;

// Flag gerada pela interrupcao externa INT1.
// A ISR apenas marca o evento; o loop principal executa a troca de estado.
volatile unsigned char flag_iniciar_analise = 0;

// Armazena o ultimo caractere recebido pela UART.
// A interrupcao USART_RX_vect recebe o byte e o loop processa depois.
volatile unsigned char comando_uart = 0;

// Guarda o instante em que o sistema entrou no ESTADO3.
// A partir desse valor, o codigo calcula quando passaram 20 segundos.
unsigned long inicio_verde = 0;

// Estado atual da maquina de estados.
// O sistema inicia bloqueado, portanto comeca em ESTADO1.
unsigned char estado = ESTADO1;

// Variavel usada para enviar mensagem de status apenas quando o estado muda.
// Isso evita ficar repetindo a mesma mensagem no terminal serial sem parar.
unsigned char ultimo_estado_enviado = 0;

/*

CONFIGURACAO DA UART:

Pinos usados automaticamente pela USAT0:
- RXD: PD0 / pino digital 0.
- TXD: PD1 / pino digital 1.

*/

void UART_Init(void)
{
   
    // Para 9600 bps com F_CPU = 16 MHz, o valor fica 103.
    UBRR0H = (unsigned char)(UBRR_VALUE >> 8);
    UBRR0L = (unsigned char)UBRR_VALUE;

    // UCSR0A contem flags de status da USART.
    // Aqui deixamos em 0 para usar o modo normal.
    UCSR0A = 0;

    // RXEN0  = habilita recepcao serial.
    // TXEN0  = habilita transmissao serial.
    // RXCIE0 = habilita interrupcao quando um byte for recebido.
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

    // UCSR0C configura o formato do quadro serial.
    //
    // UCSZ01 = 1 e UCSZ00 = 1 configuram 8 bits de dados.
    // Como UPM01/UPM00 ficam 0, nao ha paridade.
    // Como USBS0 fica 0, ha 1 bit de parada.
    //
    // Resultado: 8N1, formato padrao do monitor serial.
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

/*

ENVIO DE UM CARACTERE PELA UART

*/

void UART_Transmit(char dado)
{
    // UDRE0 indica se o registrador de transmissao UDR0 esta vazio.
    // Enquanto UDRE0 for 0, a UART ainda nao esta pronta para novo dado.
    while(!(UCSR0A & (1 << UDRE0)))
    {
        // Aguarda a UART liberar o registrador de transmissao.
    }

    // Ao escrever em UDR0, o hardware da UART transmite o byte.
    UDR0 = dado;
}

/*

ENVIO DE TEXTO PELA UART

*/

void UART_SendString(const char *texto)
{

    // O loop envia caractere por caractere ate chegar no fim da string.
    while(*texto)
    {
        UART_Transmit(*texto);
        texto++;
    }
}

/*

ENVIO DO STATUS ATUAL

*/

void UART_SendStatus(void)
{
    // Envia uma mensagem de acordo com o estado atual da maquina.
    switch(estado)
    {
        case ESTADO1:
            UART_SendString("STATUS: ACESSO BLOQUEADO\r\n");
            break;

        case ESTADO2:
            UART_SendString("STATUS: FUNCIONARIO EM ANALISE\r\n");
            break;

        case ESTADO3:
            UART_SendString("STATUS: ACESSO LIBERADO\r\n");
            break;
    }
}

/*

CONFIGURACAO DO TIMER0

Objetivo:
Gerar uma interrupcao a cada 1 ms.

Calculo:
Clock do ATmega328P = 16 MHz
Prescaler escolhido = 64

Frequencia do timer:
16 MHz / 64 = 250000 Hz

Periodo de cada contagem:
1 / 250000 = 4 us

Para chegar a 1 ms:
1 ms / 4 us = 250 contagens

Como o timer conta de 0 ate OCR0A:
OCR0A = 249 gera 250 contagens.

Modo usado: Comparação.

Quando TCNT0 chega em OCR0A, ocorre a interrupcao e o contador reinicia.

*/

void ConfigTimer0(void)
{
    // TCCR0A controla o modo de operacao do Timer0.
    // WGM01 = 1 e WGM00 = 0 configuram o modo Comparação.
    TCCR0A = (1 << WGM01);

    // TCCR0B define o prescaler.
    // CS01 = 1 e CS00 = 1 resultam em prescaler 64.
    TCCR0B = (1 << CS01) | (1 << CS00);

    // Valor de comparacao para gerar interrupcao a cada 1 ms.
    OCR0A = 249;

    // TIMSK0 habilita interrupcoes do Timer0.
    // OCIE0A = Output Compare Match A Interrupt Enable.
    TIMSK0 = (1 << OCIE0A);
}

/*

CONFIGURACAO DA INTERRUPCAO EXTERNA INT1


No ATmega328P:
- INT0 fica no PD2.
- INT1 fica no PD3.

Neste projeto, o BOTAO1 esta no PD3, portanto ele usa INT1.

O BOTAO1 esta com pull-up interno.
Assim:
- Solto: nivel alto.
- Pressionado: nivel baixo.

Quando o botao e pressionado, ocorre uma transicao de 1 para 0,
ou seja, uma borda de descida.
=========================================================
*/

void ConfigInterrupcaoExterna(void)
{
    // EICRA configura o tipo de disparo das interrupcoes externas.
    //
    // Para INT1:
    // ISC11 = 1 e ISC10 = 0 -> interrupcao na borda de descida.
    EICRA |= (1 << ISC11);
    EICRA &= ~(1 << ISC10);

    // EIFR guarda flags de interrupcao externa.
    // Escrever 1 em INTF1 limpa uma possivel flag pendente de INT1.
    EIFR = (1 << INTF1);

    // EIMSK habilita as interrupcoes externas.
    // INT1 = 1 habilita a interrupcao externa ligada ao PD3.
    EIMSK |= (1 << INT1);
}

/*

PROCESSAMENTO DOS COMANDOS RECEBIDOS PELA UART


Esta funcao implementa os comandos recebidos pelo terminal serial.

Por exemplo:
- O comando '2' so aprova se o sistema estiver no ESTADO2.
- Se o sistema estiver bloqueado, nao faz sentido aprovar ainda.
=========================================================
*/

void ProcessaComandoUART(unsigned char comando)
{
    switch(comando)
    {
        case '1':
            // Comando '1': iniciar analise.
            // So e aceito se o sistema estiver bloqueado.
            if(estado == ESTADO1)
            {
                estado = ESTADO2;
                UART_SendString("UART: ANALISE INICIADA\r\n");
            }
            else
            {
                UART_SendString("UART: COMANDO INVALIDO NESTE ESTADO\r\n");
            }
            break;

        case '2':
            // Comando '2': aprovar EPI.
            // So e aceito durante a analise.
            if(estado == ESTADO2)
            {
                estado = ESTADO3;

                // Marca o instante de entrada no estado liberado.
                // Depois o loop usa esse valor para contar 20 segundos.
                inicio_verde = tempo_ms;

                UART_SendString("UART: EPI APROVADO, ACESSO LIBERADO\r\n");
            }
            else
            {
                UART_SendString("UART: COMANDO INVALIDO NESTE ESTADO\r\n");
            }
            break;

        case '3':
            // Comando '3': reprovar EPI.
            // So e aceito durante a analise.
            if(estado == ESTADO2)
            {
                estado = ESTADO1;
                UART_SendString("UART: EPI REPROVADO, ACESSO BLOQUEADO\r\n");
            }
            else
            {
                UART_SendString("UART: COMANDO INVALIDO NESTE ESTADO\r\n");
            }
            break;

        case 's':
        case 'S':
            // Comando 's' ou 'S': apenas consulta o estado atual.
            UART_SendStatus();
            break;

        case '\r':
        case '\n':
            // O monitor serial pode enviar '\r' e/ou '\n' ao pressionar Enter.
            // Esses caracteres sao ignorados para nao aparecer erro de comando.
            break;

        default:
            // Qualquer outro caractere mostra a lista de comandos validos.
            UART_SendString("UART: COMANDOS: 1=INICIAR, 2=APROVAR, 3=REPROVAR, S=STATUS\r\n");
            break;
    }
}

/*

INTERRUPCAO DO TIMER0
Esta rotina e executada automaticamente a cada 1 ms.

*/

ISR(TIMER0_COMPA_vect)
{
    // Incrementa o contador global de tempo.
    // Essa variavel substitui o uso de delay ou _delay_ms().
    tempo_ms++;
}

/*

INTERRUPCAO EXTERNA INT1 - BOTAO1


Esta interrupcao ocorre quando o BOTAO1 e pressionado.
Como o botao usa pull-up, o pressionamento gera borda de descida.

*/

ISR(INT1_vect)
{
    // Copia o tempo atual para comparar com o ultimo acionamento aceito.
    unsigned long agora = tempo_ms;

    // Debounce simples:
    // so aceita o botao se passaram pelo menos DEBOUNCE_MS desde o ultimo.
    if((agora - ultimo_botao1_ms) >= DEBOUNCE_MS)
    {
        // Sinaliza ao loop principal que deve iniciar a analise.
        flag_iniciar_analise = 1;

        // Atualiza o instante do ultimo acionamento aceito.
        ultimo_botao1_ms = agora;
    }
}

/*

INTERRUPCAO DE RECEPCAO DA UART

Sempre que um caractere chega pela serial, o hardware gera essa interrupcao.
O caractere recebido fica no registrador UDR0.
=========================================================
*/

ISR(USART_RX_vect)
{
    // Le UDR0 para capturar o caractere recebido.
    // A leitura tambem limpa a flag de recepcao do hardware.
    comando_uart = UDR0;
}

/*

FUNCAO SETUP

Executada uma vez quando o microcontrolador inicia.
Aqui ficam as configuracoes dos pinos, timer, UART e interrupcoes.

*/

void setup()
{
    /*
    
    CONFIGURACAO DOS LEDS COMO SAIDA
    

    DDRD e o registrador de direcao do PORTD.
    Bit = 1 -> pino como saida.
    Bit = 0 -> pino como entrada.

    */
    DDRD |= (LED_VERMELHO | LED_AMARELO | LED_VERDE);

    /*
   
    CONFIGURACAO DOS BOTOES COMO ENTRADA
   
   */
    DDRD &= ~(BOTAO1 | BOTAO2 | BOTAO3);

    /*
   
    ATIVACAO DOS PULL-UPS INTERNOS
    -----------------------------------------------------

    Quando o pino esta configurado como entrada, escrever 1 em PORTD
    ativa o resistor de pull-up interno.

     */
    PORTD |= (BOTAO1 | BOTAO2 | BOTAO3);

    // Inicializa a comunicacao serial UART.
    UART_Init();

    // Configura o Timer0 para gerar uma base de tempo de 1 ms.
    ConfigTimer0();

    // Configura a interrupcao externa INT1 no BOTAO1.
    ConfigInterrupcaoExterna();

    /*
    
    ESTADO INICIAL DOS LEDS
    
    O sistema inicia bloqueado:
    - Vermelho ligado.
    - Amarelo desligado.
    - Verde desligado.
    */
    PORTD &= ~(LED_AMARELO | LED_VERDE);
    PORTD |= LED_VERMELHO;

    /*
    
    HABILITACAO GLOBAL DAS INTERRUPCOES
    
    sei() ativa a interrupção do AVR.
    
    */
    sei();

    // Mensagens iniciais exibidas no terminal serial.
    UART_SendString("Sistema iniciado\r\n");
    UART_SendString("Comandos: 1=INICIAR, 2=APROVAR, 3=REPROVAR, S=STATUS\r\n");
    UART_SendStatus();
}

/*

FUNCAO LOOP

*/

void loop()
{
  
    if(comando_uart != 0)
    {
        // Copia o comando para uma variavel local.
        unsigned char comando = comando_uart;

        // Limpa a variavel global para indicar que o comando foi lido.
        comando_uart = 0;

        // Processa o comando recebido.
        ProcessaComandoUART(comando);
    }

    /*
    -----------------------------------------------------
    ENVIO AUTOMATICO DE STATUS
    -----------------------------------------------------

    Quando o estado muda, envia uma mensagem pela UART.
    Isso ajuda na apresentacao e mostra a comunicacao serial funcionando.
    */
    if(ultimo_estado_enviado != estado)
    {
        ultimo_estado_enviado = estado;
        UART_SendStatus();
    }

    /*
    
    MAQUINA DE ESTADOS
    
    */
    switch(estado)
    {
        /*
        =================================================
        ESTADO1 - ACESSO BLOQUEADO
        =================================================

        Saidas:
        - LED vermelho ligado.
        - LED amarelo desligado.
        - LED verde desligado.

        Transicao:
        - BOTAO1 pressionado gera INT1 e ativa flag_iniciar_analise.
        - Comando UART '1' tambem pode levar ao ESTADO2.
        */
        case ESTADO1:
            // Desliga amarelo e verde.
            PORTD &= ~(LED_AMARELO | LED_VERDE);

            // Liga vermelho.
            PORTD |= LED_VERMELHO;

            // Se a interrupcao do BOTAO1 sinalizou inicio de analise,
            // muda para ESTADO2.
            if(flag_iniciar_analise)
            {
                // Limpa a flag para nao repetir a transicao.
                flag_iniciar_analise = 0;

                // Vai para o estado de analise.
                estado = ESTADO2;

                // Informa pelo terminal serial.
                UART_SendString("BOTAO1: ANALISE INICIADA\r\n");
            }
            break;

        /*
       
        ESTADO2 - FUNCIONARIO EM ANALISE
       
        Saidas:
        - LED amarelo ligado.
        - LED vermelho desligado.
        - LED verde desligado.

        Transicoes:
        - BOTAO2 ou UART '2': aprova EPI e vai para ESTADO3.
        - BOTAO3 ou UART '3': reprova EPI e volta para ESTADO1.
        */
        case ESTADO2:
            // Desliga vermelho e verde.
            PORTD &= ~(LED_VERMELHO | LED_VERDE);

            // Liga amarelo.
            PORTD |= LED_AMARELO;

            // Se BOTAO1 for pressionado de novo durante a analise,
            // a flag e descartada, pois nao faz sentido reiniciar a analise.
            flag_iniciar_analise = 0;

            // BOTAO2 pressionado?
            // Com pull-up, pressionado significa nivel 0.
            if(!(PIND & BOTAO2))
            {
                // Vai para acesso liberado.
                estado = ESTADO3;

                // Marca o tempo em que o LED verde comecou.
                inicio_verde = tempo_ms;

                // Informa a aprovacao pela UART.
                UART_SendString("BOTAO2: EPI APROVADO, ACESSO LIBERADO\r\n");
            }
            // BOTAO3 pressionado?
            else if(!(PIND & BOTAO3))
            {
                // Volta para acesso bloqueado.
                estado = ESTADO1;

                // Informa a reprovacao pela UART.
                UART_SendString("BOTAO3: EPI REPROVADO, ACESSO BLOQUEADO\r\n");
            }
            break;

        /*
        
        ESTADO3 - ACESSO LIBERADO
        
        Saidas:
        - LED verde ligado.
        - LED vermelho desligado.
        - LED amarelo desligado.

        Transicao:
        - Depois de 20 segundos, retorna automaticamente para ESTADO1.
        */
        case ESTADO3:
            // Desliga vermelho e amarelo.
            PORTD &= ~(LED_VERMELHO | LED_AMARELO);

            // Liga verde.
            PORTD |= LED_VERDE;

            // Se BOTAO1 for pressionado durante o acesso liberado,
            // a flag e descartada. O sistema so aceita nova analise
            // depois de voltar ao ESTADO1.
            flag_iniciar_analise = 0;

            // Verifica se ja passaram 20 segundos desde a entrada no ESTADO3.
            if((tempo_ms - inicio_verde) >= TEMPO_LIBERADO_MS)
            {
                // Termina o tempo de liberacao e volta ao estado bloqueado.
                estado = ESTADO1;

                // Informa que o retorno foi causado pelo timer.
                UART_SendString("TIMER: TEMPO ESGOTADO, ACESSO BLOQUEADO\r\n");
            }
            break;
    }
}

