#ifndef IMU
#define IMU
/*

@brief 
@parameters
@return 
*/

#define IMU_ACCEL_CONFIG    0x21
#define IMU_WOM_X_THR        0x4B
#define IMU_WOM_Y_THR        0x4C
#define IMU_WOM_Z_THR        0x4D
#define IMU_INT_CONFIG       0x06
#define IMU_WOM_CONFIG       0x27
#define IMU_INT_SOURCE       0x2C
#define IMU_INT_CONFIG       0x06
#define IMU_INT_STATUS       0x3B

/**
 * @brief 
 * Configurates IMU for low power interrupt operation
 * 
 * @details 
 *  Control config registers found in data sheet
 * https://invensense.tdk.com/wp-content/uploads/2021/07/DS-000451-ICM-42670-P-v1.0.pdf
 */
void IMU_LP_mode();

void test_write();
/**
 * @brief 
 * Loads threshold calibration values from flash
 */
void load_calib_IMU();


/**
 * @brief 
 * Sets calibration values according to inputs between button presses
 * 
 * @return 
 * Success on 0, error on 1
 */
int set_calib_IMU();


/**
 * @brief 
 * Reads Input from IMU and processes the strongest for return
 * 
 * @return 
 * Char depending on the strongest input direction
 * 
 */
char read_IMU();

/**
 * @brief 
 * Parses the IMU inputs for the strongest acceleration direction
 * @param ax
 * float value for the maxmimum accelerator value for x
 * @param az
 * float value for the maxmimum accelerator value for z
 * 
 * @return 
 * Char depending on the strongest input
 */
char parseIMU(float ax, float ay, float  az);

void motion_handler(uint gpio, uint32_t events);
void IMU_init();
void IMU_LP_init();
void delay_ms(uint32_t ms);
#endif /* IMU.h */