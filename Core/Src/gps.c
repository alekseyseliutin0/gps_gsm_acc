#include "gps.h"
#include <stdlib.h>
#include <string.h>

uint8_t gps_data[200];
uint8_t gps_pack[200];
GPS_INFO gps_info;

static bool GetField(uint8_t *pData, uint8_t *pField, uint8_t nFieldNum, uint8_t nMaxFieldLen);


void Check_GPS_Pack(void)
{
	uint8_t crc=0;
	uint8_t i=1;

	while(gps_pack[i]!='*')
	{
		crc^=gps_pack[i++];
		if (i>=100)
		{
			gps_info.error_crc++;
			return;
		}
	}

	if (crc == (HexASCII_to_Hex(gps_pack[i+1])<<4 | HexASCII_to_Hex(gps_pack[i+2])))
	{
		if(!memcmp((const char*)&gps_pack[1],"GNRMC",5))
			GPS_Parse_GNGGA();
	}else
	  gps_info.error_crc++;
}


//Преобразование 'F' в 0x0F
uint8_t HexASCII_to_Hex(const uint8_t HA)
{
    if (HA>='0' && HA<='9') return HA&0x0F;
    if (HA>='A' && HA<='F') return HA-0x37;
    if (HA>='a' && HA<='f') return HA-0x57;
    return 0xF0;
}

void GPS_Parse_GNGGA(void)
{
	#define MAXFIELD 16
    unsigned char pField[MAXFIELD];


    if (GetField(&gps_pack[7], pField, 0, MAXFIELD))
    {
        // Hours
        gps_info.time_pack.hour = (pField[0]&0x0F)*10 + (pField[1]&0x0F);
        // Minutes
        gps_info.time_pack.min = (pField[2]&0x0F)*10 + (pField[3]&0x0F);
        // Seconds
        gps_info.time_pack.sec = (pField[4]&0x0F)*10 + (pField[5]&0x0F);
    }

    // Data valid
    gps_info.fix_valid = GetField(&gps_pack[7], pField, 1, MAXFIELD) && pField[0] == 'A';

    // Latitude
    if (GetField(&gps_pack[7], pField, 2, MAXFIELD))
    {
        gps_info.Position.Latitude = atof((char *)pField+2)/60.0;
        gps_info.Position.Latitude += (float)((pField[0]&0x0F)*10+(pField[1]&0x0F));
    }
    if (GetField(&gps_pack[7], pField, 3, MAXFIELD))
    {
        if (pField[0] == 'S')
        gps_info.Position.Latitude = -gps_info.Position.Latitude;
    }

    // Longitude
    if (GetField(&gps_pack[7], pField, 4, MAXFIELD))
    {
        gps_info.Position.Longitude = atof((char *)pField+3)/60.0;
        gps_info.Position.Longitude += (float)((pField[0]&0x0F)*100 + (pField[1]&0x0F)*10 + (pField[2]&0x0F));
    }
    if (GetField(&gps_pack[7], pField, 5, MAXFIELD))
    {
        if (pField[0] == 'W')
        gps_info.Position.Longitude = -gps_info.Position.Longitude;
    }

    // Ground speed
    if (GetField(&gps_pack[7], pField, 6, MAXFIELD))
    {
        gps_info.Velocity.Velocity = atof((char *)pField)*0.5144444;
    }
    else
    {
        gps_info.Velocity.Velocity = 0.0;
    }

    // Course over ground, degrees true
    if (GetField(&gps_pack[7], pField, 7, MAXFIELD))
    {
        gps_info.Velocity.Course = atof((char *)pField);
    }
    else
    {
        gps_info.Velocity.Course = 0.0;
    }

    // Date
    if (GetField(&gps_pack[7], pField, 8, MAXFIELD))
    {
        // Day
        gps_info.time_pack.date = (pField[0]&0x0F)*10 + (pField[1]&0x0F);
        // Month
        gps_info.time_pack.month = (pField[2]&0x0F)*10 + (pField[3]&0x0F);
        // Year (Only two digits. I wonder why?)
        gps_info.time_pack.year = (pField[4]&0x0F)*10 + (pField[5]&0x0F);
    }


}

static bool GetField(uint8_t *pData, uint8_t *pField, uint8_t nFieldNum, uint8_t nMaxFieldLen)
{
    if(pData == NULL || pField == NULL || nMaxFieldLen == 0)
    {
        return false;
    }

    uint8_t i = 0;
    uint8_t nField = 0;
    while (nField != nFieldNum && pData[i])
    {
        if (pData[i] == ',')
        {
    	      nField++;
        }

        i++;

        if (pData[i] == 0)
        {
            pField[0] = '\0';
            return false;
        }
    }

    if (pData[i] == ',' || pData[i] == '*')
    {
        pField[0] = '\0';
        return false;
    }

    unsigned char i2 = 0;
    while (pData[i] != ',' && pData[i] != '*' && pData[i])
    {
        pField[i2] = pData[i];
        i2++;
        i++;

        if (i2 >= nMaxFieldLen)
        {
            i2 = nMaxFieldLen-1;
            break;
        }
    }

    pField[i2] = '\0';
    return true;
}
