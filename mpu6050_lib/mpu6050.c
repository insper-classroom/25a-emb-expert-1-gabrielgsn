#include "mpu6050.h"
#include "hardware/gpio.h" // Para gpio_pull_up, gpio_set_function
#include "pico/time.h"     // Para sleep_ms

// Timeout padrão para operações I2C (em microssegundos)
#define MPU6050_I2C_TIMEOUT_US 10000 // 10 ms

// ---- Funções auxiliares estáticas para comunicação I2C ----

/**
 * @brief Escreve um único byte em um registrador do MPU6050.
 * @param inst Ponteiro constante para a estrutura da instância do MPU6050.
 * @param reg Endereço do registrador a ser escrito.
 * @param data Byte a ser escrito no registrador.
 * @return 1 em sucesso, 0 em falha (timeout ou erro I2C).
 */
static int mpu6050_write_reg(const mpu6050_inst_t *inst, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    int bytes_written = i2c_write_timeout_us(inst->i2c_port, inst->i2c_addr, buf, 2, false, MPU6050_I2C_TIMEOUT_US);
    return (bytes_written == 2) ? 1 : 0;
}

/**
 * @brief Lê múltiplos bytes de registradores consecutivos do MPU6050.
 * @param inst Ponteiro constante para a estrutura da instância do MPU6050.
 * @param reg_start Endereço do primeiro registrador a ser lido.
 * @param buffer Ponteiro para o buffer onde os dados lidos serão armazenados.
 * @param len Número de bytes a serem lidos.
 * @return 1 em sucesso, 0 em falha (timeout ou erro I2C).
 */
static int mpu6050_read_regs(const mpu6050_inst_t *inst, uint8_t reg_start, uint8_t *buffer, size_t len) {
    // Envia o endereço do registrador inicial (com 'nostop' = true para manter a transação I2C ativa)
    uint8_t reg_buf = reg_start;
    int write_res = i2c_write_timeout_us(inst->i2c_port, inst->i2c_addr, &reg_buf, 1, true, MPU6050_I2C_TIMEOUT_US);
    if (write_res != 1) {
        // printf("MPU6050: Falha ao endereçar registrador 0x%02X para leitura\n", reg_start);
        return 0; // Falha ao endereçar o registrador
    }

    // Lê os dados
    int bytes_read = i2c_read_timeout_us(inst->i2c_port, inst->i2c_addr, buffer, len, false, MPU6050_I2C_TIMEOUT_US);
    if (bytes_read != (int)len) {
        // printf("MPU6050: Falha ao ler %u bytes do registrador 0x%02X (lidos: %d)\n", len, reg_start, bytes_read);
        return 0; // Falha na leitura
    }
    return 1;
}

// ---- Implementação das funções públicas do driver ----

void mpu6050_set_config(mpu6050_inst_t *inst,
                        i2c_inst_t *i2c_port,
                        uint sda_pin,
                        uint scl_pin,
                        uint8_t i2c_address,
                        uint8_t accel_fs_sel,
                        uint8_t gyro_fs_sel) {
    inst->i2c_port = i2c_port;
    inst->sda_pin = sda_pin;
    inst->scl_pin = scl_pin;
    inst->i2c_addr = i2c_address;
    inst->current_accel_fs_sel = accel_fs_sel;
    inst->current_gyro_fs_sel = gyro_fs_sel;

    // Calcula fatores de escala com base na seleção de Full Scale
    // Esses valores vêm da tabela de sensibilidade no datasheet do MPU6050
    switch (accel_fs_sel) {
        case MPU6050_ACCEL_FS_SEL_2G:  inst->accel_scale_factor = 1.0f / 16384.0f; break;
        case MPU6050_ACCEL_FS_SEL_4G:  inst->accel_scale_factor = 1.0f / 8192.0f;  break;
        case MPU6050_ACCEL_FS_SEL_8G:  inst->accel_scale_factor = 1.0f / 4096.0f;  break;
        case MPU6050_ACCEL_FS_SEL_16G: inst->accel_scale_factor = 1.0f / 2048.0f;  break;
        default: inst->accel_scale_factor = 1.0f; // Fallback, idealmente o valor de FS_SEL deve ser válido
    }

    switch (gyro_fs_sel) {
        case MPU6050_GYRO_FS_SEL_250DPS:  inst->gyro_scale_factor = 1.0f / 131.0f; break;
        case MPU6050_GYRO_FS_SEL_500DPS:  inst->gyro_scale_factor = 1.0f / 65.5f;  break;
        case MPU6050_GYRO_FS_SEL_1000DPS: inst->gyro_scale_factor = 1.0f / 32.8f;  break;
        case MPU6050_GYRO_FS_SEL_2000DPS: inst->gyro_scale_factor = 1.0f / 16.4f;  break;
        default: inst->gyro_scale_factor = 1.0f; // Fallback
    }
}

int mpu6050_reset(const mpu6050_inst_t *inst) {
    // Seta o bit DEVICE_RESET no registrador PWR_MGMT_1
    if (!mpu6050_write_reg(inst, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_DEVICE_RESET)) {
        return 0; // Falha ao escrever no registrador
    }
    // O datasheet sugere um tempo para o reset se propagar (e.g., 100ms).
    // O bit DEVICE_RESET se auto-limpa após o reset.
    sleep_ms(100); // Tempo para o dispositivo resetar completamente
    return 1;
}

int mpu6050_init(mpu6050_inst_t *inst) {
    // 1. Inicializa I2C na Pico
    // A frequência pode ser 100kHz (padrão) ou 400kHz (rápido) para o MPU6050
    i2c_init(inst->i2c_port, 100 * 1000); // 100kHz
    gpio_set_function(inst->sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(inst->scl_pin, GPIO_FUNC_I2C);
    // Habilita pull-ups internos nos pinos I2C.
    // Resistores pull-up externos (e.g., 2.2k a 10k) são geralmente recomendados para I2C,
    // mas os internos podem funcionar para curtas distâncias e baixas velocidades.
    gpio_pull_up(inst->sda_pin);
    gpio_pull_up(inst->scl_pin);

    // 2. Tenta resetar o dispositivo primeiro
    // Isso coloca o MPU6050 em um estado conhecido.
    if (!mpu6050_reset(inst)) {
        // printf("MPU6050: Falha no reset inicial.\n");
        return 0; // Falha no reset
    }

    // 3. Verifica o registrador WHO_AM_I para confirmar a comunicação e identidade do dispositivo
    uint8_t who_am_i_val;
    if (!mpu6050_read_regs(inst, MPU6050_REG_WHO_AM_I, &who_am_i_val, 1)) {
        // printf("MPU6050: Falha ao ler WHO_AM_I.\n");
        return 0; // Falha ao ler WHO_AM_I
    }
    // O valor do registrador WHO_AM_I (0x75) é tipicamente 0x68 para MPU6050.
    // Este valor é fixo e não depende do pino AD0.
    // O endereço I2C usado para comunicação (inst->i2c_addr) é que depende do AD0.
    if (who_am_i_val != MPU6050_WHO_AM_I_VAL) { // MPU6050_WHO_AM_I_VAL deve ser 0x68
        // printf("MPU6050: WHO_AM_I valor incorreto. Esperado 0x%02X, lido 0x%02X.\n", MPU6050_WHO_AM_I_VAL, who_am_i_val);
        return 0; // Dispositivo não identificado corretamente
    }

    // 4. Acorda o MPU6050 (limpa o bit SLEEP) e seleciona a fonte de clock.
    // Usar o PLL com referência do giroscópio X (ou Y, Z) é recomendado para melhor estabilidade.
    // 0x00 (CLK_INTERNAL) também é uma opção, mas menos estável.
    if (!mpu6050_write_reg(inst, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_CLK_PLL_X)) {
        // printf("MPU6050: Falha ao configurar PWR_MGMT_1 (acordar e clk src).\n");
        return 0;
    }
    sleep_ms(10); // Pequena espera para o clock estabilizar

    // 5. Configura Sample Rate Divider (SMPLRT_DIV)
    // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    // Gyroscope Output Rate = 8kHz (quando DLPF desabilitado) ou 1kHz (quando DLPF habilitado).
    // Vamos assumir que DLPF será habilitado (comum).
    // Para uma taxa de amostragem de 1kHz (máxima com DLPF): SMPLRT_DIV = 0
    // Para 500Hz: SMPLRT_DIV = 1
    // Para 200Hz: SMPLRT_DIV = 4
    if (!mpu6050_write_reg(inst, MPU6050_REG_SMPLRT_DIV, 0x00)) { // Ex: 1kHz sample rate (se Gyro Output Rate = 1kHz)
        // printf("MPU6050: Falha ao configurar SMPLRT_DIV.\n");
        return 0;
    }

    // 6. Configura DLPF (Digital Low Pass Filter) através do registrador CONFIG
    // DLPF_CFG controla a largura de banda do acelerômetro e giroscópio.
    // Ex: DLPF_CFG = 0 (Accel BW 260Hz, Gyro BW 256Hz) - Menor delay, mais ruído.
    // Ex: DLPF_CFG = 1 (Accel BW 184Hz, Gyro BW 188Hz)
    // Ex: DLPF_CFG = 6 (Accel BW 5Hz, Gyro BW 5Hz) - Maior delay, menos ruído.
    // Escolha conforme a aplicação. 0x00 é um bom padrão para começar.
    if (!mpu6050_write_reg(inst, MPU6050_REG_CONFIG, 0x00)) { // DLPF_CFG = 0 (sem filtro ou filtro mais largo)
        // printf("MPU6050: Falha ao configurar CONFIG (DLPF).\n");
        return 0;
    }

    // 7. Configura o fundo de escala do Giroscópio (GYRO_CONFIG)
    // O valor já está em inst->current_gyro_fs_sel, definido por mpu6050_set_config.
    if (!mpu6050_write_reg(inst, MPU6050_REG_GYRO_CONFIG, inst->current_gyro_fs_sel)) {
        // printf("MPU6050: Falha ao configurar GYRO_CONFIG.\n");
        return 0;
    }

    // 8. Configura o fundo de escala do Acelerômetro (ACCEL_CONFIG)
    // O valor já está em inst->current_accel_fs_sel, definido por mpu6050_set_config.
    if (!mpu6050_write_reg(inst, MPU6050_REG_ACCEL_CONFIG, inst->current_accel_fs_sel)) {
        // printf("MPU6050: Falha ao configurar ACCEL_CONFIG.\n");
        return 0;
    }

    // Outras configurações opcionais (não incluídas aqui para simplicidade):
    // - INT_PIN_CFG (configuração do pino de interrupção)
    // - INT_ENABLE (habilitação de interrupções específicas)
    // - Motion detection (WOM - Wake on Motion)

    return 1; // Sucesso na inicialização
}

int mpu6050_read_raw_accel(const mpu6050_inst_t *inst, int16_t accel[3]) {
    uint8_t buffer[6]; // ACCEL_XOUT_H, L, YOUT_H, L, ZOUT_H, L
    if (!mpu6050_read_regs(inst, MPU6050_REG_ACCEL_XOUT_H, buffer, 6)) {
        return 0; // Falha na leitura
    }
    // Os dados são Big Endian (MSB primeiro)
    accel[0] = (int16_t)((buffer[0] << 8) | buffer[1]); // X
    accel[1] = (int16_t)((buffer[2] << 8) | buffer[3]); // Y
    accel[2] = (int16_t)((buffer[4] << 8) | buffer[5]); // Z
    return 1;
}

int mpu6050_read_raw_gyro(const mpu6050_inst_t *inst, int16_t gyro[3]) {
    uint8_t buffer[6]; // GYRO_XOUT_H, L, YOUT_H, L, ZOUT_H, L
    if (!mpu6050_read_regs(inst, MPU6050_REG_GYRO_XOUT_H, buffer, 6)) {
        return 0; // Falha na leitura
    }
    // Os dados são Big Endian
    gyro[0] = (int16_t)((buffer[0] << 8) | buffer[1]); // X
    gyro[1] = (int16_t)((buffer[2] << 8) | buffer[3]); // Y
    gyro[2] = (int16_t)((buffer[4] << 8) | buffer[5]); // Z
    return 1;
}

int mpu6050_read_raw_temp(const mpu6050_inst_t *inst, int16_t *temp_raw) {
    uint8_t buffer[2]; // TEMP_OUT_H, L
    if (!mpu6050_read_regs(inst, MPU6050_REG_TEMP_OUT_H, buffer, 2)) {
        return 0; // Falha na leitura
    }
    // Os dados são Big Endian
    *temp_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    return 1;
}

float mpu6050_convert_temp_c(int16_t temp_raw) {
    // Fórmula do datasheet: Temperature in degrees C = (TEMP_OUT Register Value / 340) + 36.53
    return ((float)temp_raw / 340.0f) + 36.53f;
}

void mpu6050_convert_accel_g(const mpu6050_inst_t *inst, const int16_t accel_raw[3], float accel_g[3]) {
    for (int i = 0; i < 3; i++) {
        accel_g[i] = (float)accel_raw[i] * inst->accel_scale_factor;
    }
}

void mpu6050_convert_gyro_dps(const mpu6050_inst_t *inst, const int16_t gyro_raw[3], float gyro_dps[3]) {
    for (int i = 0; i < 3; i++) {
        gyro_dps[i] = (float)gyro_raw[i] * inst->gyro_scale_factor;
    }
}