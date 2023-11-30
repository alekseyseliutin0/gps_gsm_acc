/*
 * gps.h
 *
 *  Created on: Nov 29, 2023
 *      Author: Alex
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include <stdint.h>
#include <stdbool.h>
typedef struct _GPS_TIME
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} GPS_TIME;

typedef struct _GPS_INFO
{

    GPS_TIME time_pack; // Время считанное из пакета

    uint8_t fix_valid;  // Координаты установлены

    struct
    {
        float Latitude;
        float Longitude;
        float Altitude;
    } Position;

    struct
    {
        float Velocity;
        float Course;
    } Velocity;
		uint8_t error_crc;
} GPS_INFO;


void Check_GPS_Pack(void);
uint8_t HexASCII_to_Hex(const uint8_t HA);
void GPS_Parse_GNGGA(void);


#endif /* INC_GPS_H_ */
