#ifndef _MESSAGES_H
#define _MESSAGES_H

#include <stdint.h>

#define FRAME_TYPE_AUTO_AIM 0
#define FRAME_TYPE_NAV 1
#define FRAME_TYPE_HEART_BEAT 2
#define FRAME_TYPE_OTHER 3

typedef struct _AutoAimPackage
{
	float yaw;	 				// yaw (deg)
	float pitch; 				// pitch (deg)
	bool fire;   				// 0 = no fire, 1 = fire
} AutoAimPackage;

typedef struct _NavPackage
{
	float x_vel;				// m/s 
	float y_vel;				// m/s
	float yaw_rad; 				// rad/s
	uint8_t state;				// 0 = stationary, 1 = moving, 2 = spin
} NavPackage;

typedef struct _HeartBeatPackage
{
	uint8_t _a;
	uint8_t _b;
	uint8_t _c;
	uint8_t _d;
} HeartBeatPackage;

typedef struct _PackageOut
{
	uint8_t frame_id;
	uint8_t frame_type;
	union
	{
		AutoAimPackage autoAimPackage;
		NavPackage navPackage;
		HeartBeatPackage heartBeatPackage;
	};
} PackageOut;

typedef struct __attribute__((__packed__)) _PackageIn
{
	uint8_t head;
	uint8_t ref_flags;
	float pitch;	 				// rad
	float pitch_vel; 				// rad/s
	float yaw_vel;	 				// rad/s (ccw: +, cw: -)

	float x;		   				// m
	float y;		  				// m
	float orientation; 				// rad (ccw: +, cw: -)

	float x_vel; 					// m/s
	float y_vel; 					// m/s
} __attribute__((packed)) PackageIn;

#endif // _MESSAGES_H
