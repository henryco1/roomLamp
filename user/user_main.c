/* 
Copyright 2015 <>< Charles Lohr, see LICENSE file.
*/

#include <mdns.h>

#include "mem.h"
#include "c_types.h"
#include "user_interface.h"
#include "ets_sys.h"
#include "uart.h"
#include "osapi.h"
#include "espconn.h"
#include "esp82xxutil.h"
#include "commonservices.h"
#include "pwm.h"
#include "gpio.h"

#include "ws2812_i2s.h"
#include "vars.h"
#include "pattern.h"
#include "user_config.h"

#define procTaskPrio        0
#define procTaskQueueLen    1

static volatile os_timer_t some_timer;
static volatile os_timer_t pattern_timer;
static struct espconn *pUdpServer;
usr_conf_t * UsrCfg = (usr_conf_t*)(SETTINGS.UserData);
uint8_t last_leds[512*3] = {0};
uint32_t frame = 0;

//int ICACHE_FLASH_ATTR StartMDNS();

void ICACHE_FLASH_ATTR user_rf_pre_init(void) {/*nothing.*/}


/*=========================
COMMON SERVICES BACKEND
=========================*/
//Tasks that happen all the time.
os_event_t    procTaskQueue[procTaskQueueLen];
static void ICACHE_FLASH_ATTR procTask(os_event_t *events)
{
	CSTick( 0 );
	system_os_post(procTaskPrio, 0, 0 );
}


/*=========================
BRIGHTNESS PWM
=========================*/
static const uint32_t frequency = 6400000; // in hz, so 5kHz
uint32_t maxDuty = 0;

uint8_t ledDutyPercent = 10;
uint32_t ledDuty = 0;

uint8_t *pLedDutyPercent = &ledDutyPercent;
uint32_t *pLedDuty = &ledDuty;

void ICACHE_FLASH_ATTR 
init_brightness(void *args)
{
    gpio_init();
    maxDuty = (frequency * 1000)/45;

    uint32_t pwmInfo[1][3] = {
        {PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15, 15}
    };

    *pLedDuty = (uint32_t)((float)(ledDutyPercent/100.0) * (float)maxDuty);
    pwm_init(frequency, pLedDuty, 1, pwmInfo);
}

void ICACHE_FLASH_ATTR 
configure_brightness(void *args, uint32_t inputDutyCycle) 
{
	*pLedDuty = (uint32_t)((float)(inputDutyCycle/100.0) * (float)maxDuty);
	pwm_set_duty(*pLedDuty, 0);
	pwm_start();
}

/*=========================
TIMERS
=========================*/
static void ICACHE_FLASH_ATTR ledPatternTimer(void *arg)
{
    if(UsrCfg->ptrn == PTRN_NONE) return;

	if( UsrCfg->nled > 450 ) UsrCfg->nled = 450;
    int it;
    for(it=0; it<UsrCfg->nled; ++it) {
        uint32_t hex = hex_pattern( UsrCfg->ptrn, it, UsrCfg->nled, frame, UsrCfg->clr );
        last_leds[3*it+0] = (hex>>8);
        last_leds[3*it+1] = (hex);
        last_leds[3*it+2] = (hex>>16);
    }
    frame++;
    debug("Frame: %i", (int)frame);
    ws2812_push( (char*)last_leds, 3*UsrCfg->nled);
}

static void ICACHE_FLASH_ATTR commonServicesTimer(void *arg)
{
	CSTick( 1 );
}


/*=========================
ESP8266 SERVER
=========================*/
/*
Called when new led packet comes in.
*/
static void ICACHE_FLASH_ATTR
udp_server_recv(void *arg, char *pusrdata, unsigned short len)
{
    UsrCfg->ptrn = PTRN_NONE;
	struct espconn *pespconn = (struct espconn *)arg;

	uart0_sendStr("X");

	ws2812_push( pusrdata+3, len-3 );

	len -= 3;
	if( len > sizeof(last_leds) + 3 )
		len = sizeof(last_leds) + 3;
	ets_memcpy( last_leds, pusrdata+3, len );
	UsrCfg->nled = len / 3;
}

/*
sets up the connection to the home network
*/
void 
user_set_station_config(void)
{
    char ssid[32] = SSID;
    char password[64] = PASSWORD;
    struct station_config sta_conf = { 0 };

    os_memcpy(sta_conf.ssid, ssid, 32);
    os_memcpy(sta_conf.password, password, 64);
    wifi_station_set_config(&sta_conf);
}


/*=========================
MAIN EVENT LOOP
=========================*/
void ICACHE_FLASH_ATTR charrx( uint8_t c ) {/*Called from UART.*/}

void ICACHE_FLASH_ATTR umcall( void );

void user_init(void)
{
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	uart0_sendStr("\r\nesp82XX Web-GUI\r\n");
	umcall();
}

void ICACHE_FLASH_ATTR umcall( void )
{
//Uncomment this to force a system restore.
//	system_restore();

	CSSettingsLoad( 0 );
    CSPreInit();

	wifi_set_opmode(STATION_MODE);
	user_set_station_config();
	wifi_station_connect();

    pUdpServer = (struct espconn *)os_zalloc(sizeof(struct espconn));
	ets_memset( pUdpServer, 0, sizeof( struct espconn ) );
	espconn_create( pUdpServer );
	pUdpServer->type = ESPCONN_UDP;
	pUdpServer->proto.udp = (esp_udp *)os_zalloc(sizeof(esp_udp));
	pUdpServer->proto.udp->local_port = COM_PORT;
	espconn_regist_recvcb(pUdpServer, udp_server_recv);

	if( espconn_create( pUdpServer ) )
		while(1)
            uart0_sendStr( "\r\nFAULT\r\n" );

	CSInit();

	SetServiceName( "ws2812" );
	AddMDNSName( "esp82xx" );
	AddMDNSName( "ws2812" );
	AddMDNSService( "_http._tcp", "An ESP8266 Webserver", WEB_PORT );
	AddMDNSService( "_ws2812._udp", "WS2812 Driver", COM_PORT );
	AddMDNSService( "_esp82xx._udp", "ESP8266 Backend", BACKEND_PORT );

	//Add a process
	system_os_task(procTask, procTaskPrio, procTaskQueue, procTaskQueueLen);

	//Timer example
	os_timer_disarm(&some_timer);
	os_timer_setfn(&some_timer, (os_timer_func_t *)commonServicesTimer, NULL);
	os_timer_arm(&some_timer, 100, 1);

	//Pattern Timer example
	os_timer_disarm(&pattern_timer);
	os_timer_setfn(&pattern_timer, (os_timer_func_t *)ledPatternTimer, NULL);
	os_timer_arm(&pattern_timer, 20, 1); 

	ws2812_init();

	printf( "Boot Ok.\n" );

//	wifi_set_sleep_type(LIGHT_SLEEP_T);
//	wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);

	system_os_post(procTaskPrio, 0, 0 );
}


//There is no code in this project that will cause reboots if interrupts are disabled.
void EnterCritical() {}
void ExitCritical() {}


