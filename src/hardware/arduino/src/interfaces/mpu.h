

void mpu_init();

void mpu_calibrate();
void mpu_get_raw_measurements();

double mpu_calculate_angular_velocity();
double mpu_calculate_angle(double alpha_dot_radpsec, double Ts);