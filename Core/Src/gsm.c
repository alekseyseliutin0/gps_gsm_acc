#include "gsm.h"
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "cmsis_os.h"

uint8_t gsm_data[200];
extern UART_HandleTypeDef huart3;
uint8_t gsm_pack[100];
uint8_t p_w=0;
uint8_t p_r=0;
uint8_t mc60_count_ATE0;

uint32_t gsm_timeout;
uint8_t mc60_count_registration;
uint8_t name_operator[20];
uint8_t mc60_IMEI[16];
uint8_t mc60_registration;
uint8_t signal_level;


uint8_t get_gsm_string()
{
  static uint8_t p =0;
  while (p_w != p_r)
  {
    gsm_pack[p] = gsm_data[p_r++];
    if (p_r >= sizeof(gsm_data)/sizeof(gsm_data[0]))
      p_r = 0;

    if (gsm_pack[p] == '\n')
      continue;

    if (gsm_pack[p] == '\r')
    {
      if (p)
      {
       gsm_pack[p] = 0;
        p = 0;
        return 1;
      }
    }else
    {
    	if (++p >= sizeof(gsm_pack)/sizeof(gsm_pack[0]))
    		p =0;
    }

  }
  return 0;
}

uint8_t timeout(uint32_t t, uint32_t tm)
{
  return (HAL_GetTick() - t > tm);
}

void gsm_write(char* data)
{
	uint8_t len= strlen((const char*)data);
	HAL_UART_Transmit_IT(&huart3, (uint8_t*)data, len);
	gsm_timeout = HAL_GetTick();
}

uint8_t digit_read(const char *buf, char *target)
{

  uint8_t i = 0;

  while (i < 100)
  {
    if ((buf[i])>='0' && (buf[i])<='9')
    {
      target[i] = buf[i];
      i++;
    }
    else
      break;
  }
  target[i] = 0;
  return i;
}

void gsm(void)
{
	bool string = get_gsm_string();
	static uint8_t gsm_state = 0;


	  switch (gsm_state)
	  {
	    case 0:
	      gsm_write ("ATE0\r\n");
	      gsm_state = 1;
	      mc60_count_ATE0++;
	      break;

	    case 1:
	      if (string && !strcmp((const char*)gsm_pack, "OK"))
	      {
	    	  gsm_write ("AT+CMGF=1\r\n");
	          gsm_state = 2;
	      } else if (timeout(gsm_timeout, 1000))
	      {
	        if (mc60_count_ATE0 < 4)
	        {
	           gsm_state = 0;
	        }else
	        if (timeout(gsm_timeout, 6000))
	        {
	          mc60_count_ATE0 = 0;
	          gsm_state = 0;
	        }else if(timeout(gsm_timeout, 1500))
	        {
	         // Включить питание mc60
	        	PWR_GSM_ON;
	        }else
	        {
	         // Выключить питание mc60
	        	PWR_GSM_OFF;
	        }
	      }
	      break;

	    case 2:
	      if (string && !strcmp((const char*)gsm_pack, "OK"))
	      {
	        gsm_write ("AT+CSCS=\"GSM\"\r\n");
	        gsm_state = 3;
	      } else if (timeout(gsm_timeout, 1000))
	      {
	        gsm_state = 0;
	      }
	      break;

	    case 3:
	      if (string && (!strcmp((const char*)gsm_pack, "OK") || !strcmp((const char*)gsm_pack, "ERROR")))
	      {
	        gsm_write("AT+GSN\r\n");
	        gsm_state = 4;
	      } else if (timeout(gsm_timeout, 1000))
	        gsm_state = 0;
	      break;

	    case 4:
	      if (string && gsm_pack[0]>='0' && gsm_pack[0]<='9')
	      {
	        if (digit_read((const char*)gsm_pack, (char*)mc60_IMEI) == 15)
	        {
	          gsm_state = 5;
	        } else
	          gsm_state = 0;
	      }else if (timeout(gsm_timeout, 1000))
	        gsm_state = 0;
	      break;

	    case 5:
	      if (string && !strcmp((const char*)gsm_pack, "OK"))
	      {
	        gsm_write ("AT+CPIN?\r\n");
	        gsm_state = 6;
	      } else if (timeout(gsm_timeout, 1000))
	      {
	        gsm_state = 0;
	      }
	      break;



	    case 6:
	      if (string && !strcmp((const char*)gsm_pack, "+CPIN: READY"))
	      {
	        gsm_state = 7;
	        mc60_count_registration = 0;
	      } else if (string && !strcmp((const char*)gsm_pack, "+CPIN: SIM PIN"))
	      {
	        gsm_write("AT+CPIN=0000\r\n");
	        gsm_state = 4;
	      } else if (!p_w && !strcmp((const char*)gsm_pack, "ERROR"))
	      {
	        gsm_state = 0;
	      } else if (timeout(gsm_timeout, 1000))
	      {
	        gsm_state = 0;
	      }
	      break;


	    case 7:
	      if (string && !strcmp((const char*)gsm_pack, "OK"))
	      {
	        gsm_write("AT+CREG?\r\n");
	        gsm_state = 8;
	      } else if (timeout(gsm_timeout, 1000))
	        gsm_state = 0;
	      break;



	    case 8:
	      if (string && !strncmp((const char*)gsm_pack, "+CREG:", 6))
	      {
	        mc60_registration = gsm_pack[9] - '0';
	        if (gsm_pack[9] == '1' || gsm_pack[9] == '5')
	        {
	          gsm_state = 9;
	        } else
	        {
	          if (mc60_count_registration++ < 20)
	            gsm_state = 7;
	          else
	            gsm_state = 0;
	        }
	      } else if (timeout(gsm_timeout, 1000))
	        gsm_state = 0;
	      break;

	    case 9:
	      if (string && !strcmp((const char*)gsm_pack, "OK"))
	      {
	        gsm_write("AT+COPS?\r\n");
	        gsm_state = 10;
	      } else if (timeout(gsm_timeout, 1000))
	        gsm_state = 0;
	      break;

	    case 10:
	      if (string && !strncmp((const char*)gsm_pack, "+COPS:", 6))
	      {
	        uint8_t i = 6;
	        uint8_t n = 0;
	        while (gsm_pack[i++] != '"');
	        while (gsm_pack[i] != '"' && n<sizeof(name_operator)-1)
	        	name_operator[n++] = gsm_pack[i++];
	        name_operator[n] = 0;
	        signal_level =0;
	        gsm_write("AT+CSQ\r\n");
	        gsm_state = 11;
	      } else if (timeout(gsm_timeout, 1000))
	        gsm_state = 0;
	      break;

	    case 11:
	      if (string && !strncmp ((const char*)gsm_pack, "+CSQ:", 5))
	      {
	        if (gsm_pack[7] != ',' )
	          signal_level = (gsm_pack[6] - '0') * 10 + gsm_pack[7] - '0';
	        else
	          signal_level = gsm_pack[6] - '0';

	        gsm_write("AT+GSN\r\n");
	        gsm_state = 4;
	      } else if (timeout(gsm_timeout, 1000))
	        gsm_state = 0;
	      break;
	  }
}
