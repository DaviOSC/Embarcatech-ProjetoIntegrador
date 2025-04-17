#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "hardware/clocks.h"
#include "pio_matrix.pio.h"


// Definições do display OLED
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define BAUD_RATE 115200 // Define a taxa de transmissão

// Definições de pinos do joystick, botões e LEDs
#define VRY_PIN 26
#define VRX_PIN 27
#define SW_PIN 22
#define LED_PIN_RED 13
#define LED_PIN_GREEN 11
#define LED_PIN_BLUE 12
#define PIN_BUTTON_A 5
#define PIN_BUTTON_B 6
#define OUT_PIN 7
#define BUZZER_PIN 21

#define CALIBRATION_OFFSET 100 // Offset para calibração do joystick

#define DEBOUNCE_TIME_MS 300 // Tempo de debounce em ms

#define SQUARE_SIZE 8 // Tamanho do quadrado
#define NUM_PIXELS 25 // Número de pixels na matriz de LEDs

bool green_led_state = false; // Estado inicial do LED verde
bool blue_led_state = true; // Estado inicial do LED vermelho
bool buzzer_active = false;   // Estado inicial da ativação do LED com PWM

absolute_time_t last_interrupt_time = {0};
ssd1306_t ssd;
int color = 1;

double led_matrix[NUM_PIXELS] = {0};

void play_note(int buzzer, int frequency, int duration);
void update_led_matrix(int square_pos_x, int square_pos_y);
static void gpio_irq_handler(uint gpio, uint32_t events);
uint32_t matrix_rgb(double r, double g, double b);
void pio_drawn(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b);
uint pwm_init_gpio(uint gpio, uint wrap);
void print_screen(int square_pos_y, int square_pos_x, int color);

int main()
{
    // Inicialização do PIO
    PIO pio = pio0; 
    uint32_t valor_led;  
    stdio_init_all();

    // Configurações da PIO
    uint offset = pio_add_program(pio, &pio_matrix_program);
    uint sm = pio_claim_unused_sm(pio, true);
    pio_matrix_program_init(pio, sm, offset, OUT_PIN);
    
    // Inicializa o ADC e os pinos do joystick
    adc_init();
    adc_gpio_init(VRX_PIN);
    adc_gpio_init(VRY_PIN);
    gpio_init(SW_PIN);
    
    // Inicializa os pinos dos botões e LEDs
    gpio_init(PIN_BUTTON_A);
    gpio_init(PIN_BUTTON_B);
    gpio_init(LED_PIN_GREEN);
    gpio_init(LED_PIN_BLUE);
    gpio_init(LED_PIN_RED);

    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_clkdiv(slice_num, 4.0);
    pwm_set_wrap(slice_num, 4095);
    pwm_set_enabled(slice_num, true);

        
    gpio_set_dir(PIN_BUTTON_A, GPIO_IN);
    gpio_set_dir(PIN_BUTTON_B, GPIO_IN);
    gpio_set_dir(SW_PIN, GPIO_IN);
    gpio_set_dir(LED_PIN_GREEN, GPIO_OUT);
    gpio_set_dir(LED_PIN_BLUE, GPIO_OUT);
    gpio_set_dir(LED_PIN_RED, GPIO_OUT);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_pull_up(PIN_BUTTON_A);
    gpio_pull_up(PIN_BUTTON_B);
    gpio_pull_up(SW_PIN);

    // Configura a interrupção dos botões
    gpio_set_irq_enabled_with_callback(PIN_BUTTON_A, GPIO_IRQ_EDGE_FALL, 1, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(PIN_BUTTON_B, GPIO_IRQ_EDGE_FALL, 1, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(SW_PIN, GPIO_IRQ_EDGE_FALL, 1, &gpio_irq_handler);

    // Configura o pwm para os LEDs
    i2c_init(I2C_PORT, 400 * 1000);            // Inicializa o barramento I2C
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Configura o pino SDA
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Configura o pino SCL
    gpio_pull_up(I2C_SDA);                     // Habilita o pull-up no pino SDA
    gpio_pull_up(I2C_SCL);                     // Habilita o pull-up no pino SCL

    // Inicializa o display OLED
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    
    // Lê os valores iniciais do joystick para calibração
    adc_select_input(0);
    uint16_t vry_value = adc_read();
    adc_select_input(1);
    uint16_t vrx_value = adc_read();

    // Calibração do joystick
    uint16_t vrx_calibration = vrx_value;
    uint16_t vry_calibration = vry_value;

    // Inicializa as variáveis de posição do quadrado
    int square_pos_x = 0;
    int square_pos_y = 0;
    gpio_put(LED_PIN_GREEN, 1);

    while (true)
    {

        // Lê os valores do joystick
        adc_select_input(0);
        uint16_t vry_value = adc_read();
        adc_select_input(1);
        uint16_t vrx_value = adc_read();

        // Mapear valores do joystick para PWM
        int16_t calibrated_vrx_value = vrx_value - vrx_calibration;
        int16_t calibrated_vry_value = vry_value - vry_calibration;
        int16_t mapped_vrx_value = calibrated_vrx_value;
        int16_t mapped_vry_value = calibrated_vry_value;

        // Limita os valores mapeados
        if (mapped_vrx_value < 0)
        {
            mapped_vrx_value = -mapped_vrx_value;
        }
        if (mapped_vry_value < 0)
        {
            mapped_vry_value = -mapped_vry_value;
        }
        if (mapped_vrx_value < CALIBRATION_OFFSET && mapped_vrx_value > 0)
        {
            mapped_vrx_value = 0;
        }
        if (abs(mapped_vry_value) < CALIBRATION_OFFSET && abs(mapped_vry_value) > 0)
        {
            mapped_vry_value = 0;
        }

        // Define a posição do quadrado de acordo com os valores do joystick
        square_pos_x = (WIDTH - SQUARE_SIZE) / 2 + (calibrated_vrx_value * (WIDTH - SQUARE_SIZE) / 4095);
        square_pos_y = (HEIGHT - SQUARE_SIZE) / 2 + (-calibrated_vry_value * (HEIGHT - SQUARE_SIZE) / 4096);

        print_screen(square_pos_y, square_pos_x, color);
        // Atualiza a posição anterior do quadrado para a posição atual
        update_led_matrix(square_pos_x, square_pos_y);
        pio_drawn(led_matrix, valor_led, pio, sm, 0, color, !color);

        if (buzzer_active)
        {

            int frequency = 200 + (calibrated_vrx_value * 100 / 4095);
            if (frequency < 100) frequency = 100;

            uint wrap = 10000000 / frequency;

            pwm_set_wrap(slice_num, wrap);
            pwm_set_gpio_level(BUZZER_PIN, wrap/10); 
        }
        else
        {
            pwm_set_gpio_level(BUZZER_PIN, 0);
        }
        sleep_ms(10);
    }

    return 0;
}
// Função para converter um valor RGB para um valor de 32 bits
// A função recebe os valores de vermelho, verde e azul do LED
uint32_t matrix_rgb(double r, double g, double b)
{
  unsigned char R, G, B;
  R = r * 255;
  G = g * 255;
  B = b * 255;
  return (G << 24) | (R << 16) | (B << 8);
}

//rotina para acionar a matrix de leds
void pio_drawn(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b)
{
  for (int16_t i = 0; i < NUM_PIXELS; i++)
  {
    uint32_t valor_led = matrix_rgb(r * desenho[24 - i], g * desenho[24 - i], b * desenho[24 - i]);
    pio_sm_put_blocking(pio, sm, valor_led);
  }
}

void update_led_matrix(int square_pos_x, int square_pos_y)
{
    for (int i = 0; i < NUM_PIXELS; i++)
    {
        led_matrix[i] = 0;
    }

    // Calcula a posição na matriz 5x5
    int matrix_x = square_pos_x * 5 / WIDTH;  // Converte a posição X para a matriz 5x5
    int matrix_y = square_pos_y * 5 / HEIGHT; // Converte a posição Y para a matriz 5x5

    // Garante que os índices estão dentro dos limites
    if (matrix_x >= 0 && matrix_x < 5 && matrix_y >= 0 && matrix_y < 5)
    {
        // Inverte as colunas para as linhas 1 e 3
        if (matrix_y == 1 || matrix_y == 3)
        {
            matrix_x = 4 - matrix_x;
        }

        int index = matrix_y * 5 + matrix_x; // Calcula o índice linear no vetor
        led_matrix[index] = 0.1;            // Define o valor na posição do quadrado
    }
}

void play_note(int buzzer, int frequency, int duration) {
    if (frequency == 0)
    {
      sleep_ms(duration);  // Pausa se a frequência for 0
      return;
    }
  
    int delay = 1000000 / frequency / 2; // Meio ciclo da frequência
    int cycles = (frequency * duration) / 1000; // Número de ciclos para a duração
  
    for (int i = 0; i < cycles; i++) {
        gpio_put(buzzer, 1); // Liga o buzzer
        sleep_us(delay); // Aguarda o tempo do ciclo
        gpio_put(buzzer, 0); // Desliga o buzzer
        sleep_us(delay);// Aguarda o tempo do ciclo
    }
  }
// Inicializa o PWM para um pino GPIO específico
uint pwm_init_gpio(uint gpio, uint wrap)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, wrap);

    pwm_set_enabled(slice_num, true);
    return slice_num;
}

void print_screen(int square_pos_y, int square_pos_x, int color)
{

    ssd1306_fill(&ssd, !color);

    ssd1306_rect(&ssd, 3, 3, 122, 60, color, 0);
    ssd1306_rect(&ssd, 4, 4, 120, 58, color, 0);
    ssd1306_rect(&ssd, 5, 5, 118, 56, color, 0);

    ssd1306_rect(&ssd, square_pos_y, square_pos_x, SQUARE_SIZE, SQUARE_SIZE, color, 1);

    ssd1306_send_data(&ssd);
}
// Função de tratamento de interrupção do GPIO
static void gpio_irq_handler(uint gpio, uint32_t events)
{
    // Obter o tempo atual para o debounce
    absolute_time_t current_time = get_absolute_time();

    if (absolute_time_diff_us(last_interrupt_time, current_time) < DEBOUNCE_TIME_MS * 1000)
    {
        return; // Ignora a interrupção se estiver dentro do tempo de debounce
    }
    else
    {
        last_interrupt_time = current_time;
    }

    // Ativa ou desativa a funcionalidade do LED com PWM quando o botão A é pressionado
    if (gpio == PIN_BUTTON_A)
    {
        buzzer_active = !buzzer_active;
    }
    // Alterna o estado do LED verde e o estilo da borda do display quando o botão do joystick é pressionado
    else if (gpio == SW_PIN)
    {
        color = !color;
    }
    else if (gpio == PIN_BUTTON_B)
    {
        // Ativar o BOOTSEL
        printf("BOOTSEL ativado.\n");
        reset_usb_boot(0, 0);
    }
    // Atualiza o estado dos LEDs
    gpio_put(LED_PIN_GREEN, color);
    gpio_put(LED_PIN_BLUE, !color);
}
