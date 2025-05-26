#ifndef MPU6050_H_
#define MPU6050_H_

#include "hardware/i2c.h"
#include "pico/types.h" // Para uint e uint8_t etc.
#include <stddef.h> // Para size_t

// --- Constantes do MPU6050 ---

// Endereço I2C padrão (AD0 = LOW)
#define MPU6050_I2C_ADDR_DEFAULT 0x68
// Endereço I2C alternativo (AD0 = HIGH)
#define MPU6050_I2C_ADDR_ALT     0x69

// Registradores MPU6050
#define MPU6050_REG_SMPLRT_DIV     0x19
#define MPU6050_REG_CONFIG         0x1A
#define MPU6050_REG_GYRO_CONFIG    0x1B
#define MPU6050_REG_ACCEL_CONFIG   0x1C
#define MPU6050_REG_ACCEL_XOUT_H   0x3B
#define MPU6050_REG_ACCEL_XOUT_L   0x3C
#define MPU6050_REG_ACCEL_YOUT_H   0x3D
#define MPU6050_REG_ACCEL_YOUT_L   0x3E
#define MPU6050_REG_ACCEL_ZOUT_H   0x3F
#define MPU6050_REG_ACCEL_ZOUT_L   0x40
#define MPU6050_REG_TEMP_OUT_H     0x41
#define MPU6050_REG_TEMP_OUT_L     0x42
#define MPU6050_REG_GYRO_XOUT_H    0x43
#define MPU6050_REG_GYRO_XOUT_L    0x44
#define MPU6050_REG_GYRO_YOUT_H    0x45
#define MPU6050_REG_GYRO_YOUT_L    0x46
#define MPU6050_REG_GYRO_ZOUT_H    0x47
#define MPU6050_REG_GYRO_ZOUT_L    0x48
#define MPU6050_REG_PWR_MGMT_1     0x6B
#define MPU6050_REG_WHO_AM_I       0x75
// No seu mpu6050.h, adicione esta definição se não existir:
#define MPU6050_WHO_AM_I_VAL 0x68 // Valor esperado do registrador WHO_AM_I

// Bits do registrador PWR_MGMT_1
#define MPU6050_PWR_MGMT_1_DEVICE_RESET (1 << 7)
#define MPU6050_PWR_MGMT_1_SLEEP        (1 << 6)
#define MPU6050_PWR_MGMT_1_CLK_PLL_X    0x01 // PLL com referência do giroscópio X

// Configurações de fundo de escala do Acelerômetro (ACCEL_CONFIG)
#define MPU6050_ACCEL_FS_SEL_2G  (0b00 << 3) // +/- 2g
#define MPU6050_ACCEL_FS_SEL_4G  (0b01 << 3) // +/- 4g
#define MPU6050_ACCEL_FS_SEL_8G  (0b10 << 3) // +/- 8g
#define MPU6050_ACCEL_FS_SEL_16G (0b11 << 3) // +/- 16g

// Configurações de fundo de escala do Giroscópio (GYRO_CONFIG)
#define MPU6050_GYRO_FS_SEL_250DPS  (0b00 << 3) // +/- 250 graus/s
#define MPU6050_GYRO_FS_SEL_500DPS  (0b01 << 3) // +/- 500 graus/s
#define MPU6050_GYRO_FS_SEL_1000DPS (0b10 << 3) // +/- 1000 graus/s
#define MPU6050_GYRO_FS_SEL_2000DPS (0b11 << 3) // +/- 2000 graus/s

// Estrutura da instância do MPU6050
typedef struct {
    i2c_inst_t *i2c_port;
    uint sda_pin;
    uint scl_pin;
    uint8_t i2c_addr;

    uint8_t current_accel_fs_sel;
    uint8_t current_gyro_fs_sel;

    float accel_scale_factor; // Para converter raw para G
    float gyro_scale_factor;  // Para converter raw para DPS
} mpu6050_inst_t;


// --- Protótipos das Funções Públicas ---

/**
 * @brief Configura os parâmetros da instância do MPU6050.
 *
 * @param inst Ponteiro para a estrutura da instância do MPU6050.
 * @param i2c_port Ponteiro para a instância I2C a ser usada (e.g., i2c0, i2c1).
 * @param sda_pin Pino GPIO para SDA.
 * @param scl_pin Pino GPIO para SCL.
 * @param i2c_address Endereço I2C do MPU6050 (normalmente MPU6050_I2C_ADDR_DEFAULT).
 * @param accel_fs_sel Seleção do fundo de escala do acelerômetro (e.g., MPU6050_ACCEL_FS_SEL_2G).
 * @param gyro_fs_sel Seleção do fundo de escala do giroscópio (e.g., MPU6050_GYRO_FS_SEL_250DPS).
 */
void mpu6050_set_config(mpu6050_inst_t *inst,
                        i2c_inst_t *i2c_port,
                        uint sda_pin,
                        uint scl_pin,
                        uint8_t i2c_address,
                        uint8_t accel_fs_sel,
                        uint8_t gyro_fs_sel);

/**
 * @brief Reseta o MPU6050 para as configurações de fábrica.
 * @param inst Ponteiro para a estrutura da instância do MPU6050.
 * @return 1 em sucesso, 0 em falha.
 */
int mpu6050_reset(const mpu6050_inst_t *inst);

/**
 * @brief Inicializa o MPU6050.
 * Configura I2C, reseta o dispositivo, verifica WHO_AM_I,
 * acorda o sensor e configura taxas de amostragem, DLPF e escalas.
 * @param inst Ponteiro para a estrutura da instância do MPU6050.
 * @return 1 em sucesso, 0 em falha.
 */
int mpu6050_init(mpu6050_inst_t *inst);

/**
 * @brief Lê os dados brutos do acelerômetro (X, Y, Z).
 * @param inst Ponteiro para a estrutura da instância do MPU6050.
 * @param accel Array de 3 int16_t para armazenar os dados brutos [X, Y, Z].
 * @return 1 em sucesso, 0 em falha.
 */
int mpu6050_read_raw_accel(const mpu6050_inst_t *inst, int16_t accel[3]);

/**
 * @brief Lê os dados brutos do giroscópio (X, Y, Z).
 * @param inst Ponteiro para a estrutura da instância do MPU6050.
 * @param gyro Array de 3 int16_t para armazenar os dados brutos [X, Y, Z].
 * @return 1 em sucesso, 0 em falha.
 */
int mpu6050_read_raw_gyro(const mpu6050_inst_t *inst, int16_t gyro[3]);

/**
 * @brief Lê o dado bruto da temperatura.
 * @param inst Ponteiro para a estrutura da instância do MPU6050.
 * @param temp_raw Ponteiro para int16_t para armazenar o dado bruto da temperatura.
 * @return 1 em sucesso, 0 em falha.
 */
int mpu6050_read_raw_temp(const mpu6050_inst_t *inst, int16_t *temp_raw);

/**
 * @brief Converte o dado bruto da temperatura para graus Celsius.
 * @param temp_raw Dado bruto da temperatura.
 * @return Temperatura em graus Celsius.
 */
float mpu6050_convert_temp_c(int16_t temp_raw);

/**
 * @brief Converte os dados brutos do acelerômetro para G's.
 * @param inst Ponteiro para a estrutura da instância do MPU6050 (usado para o fator de escala).
 * @param accel_raw Array com os dados brutos do acelerômetro [X, Y, Z].
 * @param accel_g Array de 3 floats para armazenar os dados convertidos em G [X, Y, Z].
 */
void mpu6050_convert_accel_g(const mpu6050_inst_t *inst, const int16_t accel_raw[3], float accel_g[3]);

/**
 * @brief Converte os dados brutos do giroscópio para graus por segundo (DPS).
 * @param inst Ponteiro para a estrutura da instância do MPU6050 (usado para o fator de escala).
 * @param gyro_raw Array com os dados brutos do giroscópio [X, Y, Z].
 * @param gyro_dps Array de 3 floats para armazenar os dados convertidos em DPS [X, Y, Z].
 */
void mpu6050_convert_gyro_dps(const mpu6050_inst_t *inst, const int16_t gyro_raw[3], float gyro_dps[3]);


// --- Funções de Motion Detection (do seu lab, mas não implementadas no .c fornecido) ---
// Se você for implementá-las, descomente e adicione ao .c
/*
int mpu6050_set_motion_detection(const mpu6050_inst_t *inst, int enable);
int mpu6050_get_motion_interrupt_status(const mpu6050_inst_t *inst);
int mpu6050_set_motion_detection_threshold(const mpu6050_inst_t *inst, uint8_t threshold);
int mpu6050_set_motion_detection_duration(const mpu6050_inst_t *inst, uint8_t duration);
*/

#endif // MPU6050_H_