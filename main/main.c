#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h" // Para i2c_default e pinos padrão se usados
#include "mpu6050.h"

// --- Configurações do Hardware ---
// ATENÇÃO: Verifique e ajuste estes pinos conforme sua placa e ligações!
// Pinos I2C padrão do Pico (GP4 para SDA, GP5 para SCL no i2c0)
// Se usar i2c1, os pinos padrão são GP2 (SDA) e GP3 (SCL)
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4  // GPIO4 como SDA (Pico Pin 6)
#define I2C_SCL_PIN 5  // GPIO5 como SCL (Pico Pin 7)

// MPU6050 I2C Address
#define MPU6050_ADDRESS MPU6050_I2C_ADDR_DEFAULT // 0x68 (AD0 = LOW)

int main() {
    stdio_init_all(); // Inicializa USB stdio para printf

    // Pausa para dar tempo ao monitor serial de se conectar
    sleep_ms(2000);
    printf("Iniciando teste do MPU6050...\n");

    mpu6050_inst_t mpu;

    // 1. Configurar a instância do MPU6050
    mpu6050_set_config(&mpu,
                       I2C_PORT,
                       I2C_SDA_PIN,
                       I2C_SCL_PIN,
                       MPU6050_ADDRESS,
                       MPU6050_ACCEL_FS_SEL_2G,  // Fundo de escala do acelerômetro: +/- 2g
                       MPU6050_GYRO_FS_SEL_250DPS); // Fundo de escala do giroscópio: +/- 250 graus/s

    // 2. Inicializar o MPU6050
    if (!mpu6050_init(&mpu)) {
        printf("Falha na inicializacao do MPU6050!\n");
        printf("Verifique as conexoes I2C, endereco e o sensor.\n");
        while (1) {
            tight_loop_contents(); // Trava se a inicialização falhar
        }
    }
    printf("MPU6050 inicializado com sucesso!\n");

    // Variáveis para armazenar os dados
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    float accel_g[3], gyro_dps[3], temp_c;

    while (1) {
        // 3. Ler dados brutos
        if (!mpu6050_read_raw_accel(&mpu, accel_raw)) {
            printf("Erro ao ler acelerometro!\n");
        }
        if (!mpu6050_read_raw_gyro(&mpu, gyro_raw)) {
            printf("Erro ao ler giroscopio!\n");
        }
        if (!mpu6050_read_raw_temp(&mpu, &temp_raw)) {
            printf("Erro ao ler temperatura!\n");
        }

        // 4. Converter dados para unidades físicas
        mpu6050_convert_accel_g(&mpu, accel_raw, accel_g);
        mpu6050_convert_gyro_dps(&mpu, gyro_raw, gyro_dps);
        temp_c = mpu6050_convert_temp_c(temp_raw);

        // 5. Imprimir os dados
        printf("Accel: X=%.2fg, Y=%.2fg, Z=%.2fg | ", accel_g[0], accel_g[1], accel_g[2]);
        printf("Gyro: X=%.2fdps, Y=%.2fdps, Z=%.2fdps | ", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        printf("Temp: %.2f C\n", temp_c);
        
        // Imprimir dados brutos também (opcional, para debug)
        // printf("Raw Accel: X=%d, Y=%d, Z=%d | ", accel_raw[0], accel_raw[1], accel_raw[2]);
        // printf("Raw Gyro: X=%d, Y=%d, Z=%d | ", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        // printf("Raw Temp: %d\n", temp_raw);

        sleep_ms(500); // Intervalo entre as leituras
    }

    return 0; // Nunca alcançado
}