#ifndef IMU
#define IMU

void init_IMU_int();
void load_calib_IMU();
int set_calib_IMU();
char read_IMU();
char parseIMU(float *ax,float  *ay,float  *az);
#endif /* IMU.h */