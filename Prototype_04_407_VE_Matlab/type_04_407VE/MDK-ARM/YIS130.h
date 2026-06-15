#ifndef YIS130_H
#define YIS130_H
#include <stdint.h>
#define MPU_CAN   hcan1   
#define AS5048_COUNTS_PER_REV 16384.0f
#define WHEEL_DIAMETER_M 0.0818f
#define AS5048_WHEEL_METERS_PER_COUNT_RAW ((3.1415926f * WHEEL_DIAMETER_M) / AS5048_COUNTS_PER_REV)
#define AS5048_LEFT_TRAVEL_CALIBRATION 1.0f
#define AS5048_RIGHT_TRAVEL_CALIBRATION 1.0f
#define AS5048_LEFT_METERS_PER_COUNT (AS5048_WHEEL_METERS_PER_COUNT_RAW * AS5048_LEFT_TRAVEL_CALIBRATION)
#define AS5048_RIGHT_METERS_PER_COUNT (AS5048_WHEEL_METERS_PER_COUNT_RAW * AS5048_RIGHT_TRAVEL_CALIBRATION)

/* Orthogonal tracking wheel defaults: SPI1 -> body X/forward, SPI2 -> body Y/left. */
#define ODOM_X_ENCODER_INDEX 0
#define ODOM_Y_ENCODER_INDEX 1
#define ODOM_X_ENCODER_SIGN 1.0f
#define ODOM_Y_ENCODER_SIGN 1.0f
#define ODOM_X_WHEEL_METERS_PER_COUNT AS5048_RIGHT_METERS_PER_COUNT
#define ODOM_Y_WHEEL_METERS_PER_COUNT AS5048_LEFT_METERS_PER_COUNT
#define ODOM_X_WHEEL_Y_OFFSET_M 0.0f
#define ODOM_Y_WHEEL_X_OFFSET_M 0.0f


typedef struct
{
    float acc[3];
		float acc_cali[3];
		double vel[3];
	  float gyro[3];         
      
	  
	  float PITCH;
	  float YAW; 
	  float ROLL;
	  float PITCH_ANGLE;
	  float YAW_ANGLE; 
	  float ROLL_ANGLE;
	  float PITCH_ANGLE_BEG;
	  float YAW_ANGLE_BEG;
	  float ROLL_ANGLE_BEG;
	  float PITCH_ANGLE_Del;
	  float YAW_ANGLE_Del;
	  float ROLL_ANGLE_Del;



		float quat[4];
	  float ACCX_CALI;
		float ACCY_CALI;
		float ACCZ_CALI;
	float ACCX_FILTER;
	float ACCY_FILTER;
	  int cali ;
	
	 // DATA FROM ENCODER
	 float REAL_X;
	 float REAL_Y;
	 float REAL_YAW;
	 float REAL_YAW_SET;
	 float REAL_YAW_MARK;
	 float X_tt;
	 float Y_tt;
	
} MPU_DATA;




void CAN_CMD_ENCODER();


void can_filter_init(void);

void get_mpu_measure(MPU_DATA *ptr,uint8_t *data ) ;

extern MPU_DATA mpu_data[4];
extern float output_vector_data[3];

void VECTOR_CONVERT();

void SelfCalibration();

void DATARELOAD(uint8_t * arr);

extern float ACCX,ACCY,ACCZ; // ��ߵ�У׼�����Ҫ�ϳ����ܲ���
void arm_fir_f32_lp(void);

#endif
