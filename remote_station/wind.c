/*
 * $Id: wind.c,v 3.0 2013/09/10 11:33:52 az Exp az $
 * 
 * File:		wind.c
 * Date:		09 Dec 2012 14:48:56
 * Author:		Alexander Zangerl <az@snafu.priv.at>
 *
 * copyright (c) 2013 Alexander Zangerl <az@snafu.priv.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License).
 *
 * Abstract: 
 *	semi-autonomous remote wind station with 
 *	davis 7911 anemometer and sim900 gsm module
 *
 */

#ifdef __XC__
/* xc8 uses different name than sdcc */
#define NOT_TO nTO
#define NOT_POR nPOR

#include <xc.h>
#define _XTAL_FREQ 8000000
#endif

/* i don't want loads of repeated inline code around */
void delay1ms()
{
   __delay_ms(1);
}
void delay256ms()
{
   __delay_ms(256);
}

#pragma config CPD=OFF, CP=OFF, DEBUG=OFF, WRT=OFF, LVP=OFF, BOREN=OFF, \
   PWRTE=ON, MCLRE=ON,WDTE=ON, FOSC=INTOSCIO
#pragma config IESO=OFF,FCMEN=OFF

/* 
#define ISP "string with APN
#define SERVER "string with target server name or ip address"
#define PORT "string with port number"
 */

#ifndef ISP
#error you must define your ISP's APN here or via -DISP=\"...\" in the makefile
#endif
#ifndef SERVER
#error you must define your target server here or via -DSERVER=\"...\" in the makefile
#endif
#ifndef PORT
#error you must define your target port here or via -DPORT=\"...\" in the makefile
#endif


/* relevant pins:
   ra0	direction sensor in (analog pot)
   ra1	sim900 powerkey (the non-lifted one!), pull low for on/off 
   	(sim900 does pullup)
   ra2	dtr in and out (also non-lifted), to check sim900 aliveness (dtr 
   	is at +2.8v if on, but at about 1.5V if off), must be pulled low 
	before tx or cts isn't done by sim900
   ra4 	speed sensor in (tmr0 counts low-level pulses, pulled up to +5v 
   	via resistors and 100nF)
  
   rb0	ri_vin in (lifted!), ring indicator, triggers reset of the whole shebang
   rb1	diag led output
   rb2	serial rx, from sim900
   rb3	cts (non-lifted), low: ready, high: please wait
   rb4	serial tx, to sim900
   rb6	t1osc, 32.768kHz for clock
   rb7	t1osc
*/
#define POWERBIT 1
#define ALIVEBIT 2

#define RINGBIT	0
#define LEDBIT 1
#define CTSBIT 3

unsigned char initsim900(void), isalive(void), crc8(unsigned char);
void killsim900(void),reseteverything();
void sertxchar(unsigned char), sertxstring(const unsigned char *), delayfive(unsigned char);

    
static volatile unsigned int uptime;		/* incd every 5s */
static volatile unsigned char seconds;		/* 0..4, updated per second */
static volatile unsigned char intblinky;
static unsigned char speed, dir[5], crc;

/* initsim900: sets ininit when starting its work, and clears it once finished 
   ignwdt: if set then wdt reset does not cause a power-down of the sim900 
   on start 
   wdtexp: set if watchdog did expire 
   respfound: set when send ok found, clear otherwise */
static volatile struct { 
   unsigned char ininit:1, ignwdt:1, wdtexp:1, okrespfound:1;
} status; 

const unsigned char respstring[]="\r\nSEND OK";
static volatile unsigned char respnextchar;
#define MAXCONSEC 24		/* 24x5=120sec consecutive send errors and we reset */

#define clrwdt asm("clrwdt");

void main(void)
{
   unsigned char i,foo;
   
   status.wdtexp=!NOT_TO;	/* timeout flag is reset by clrwdt */
   
   /* prime the watchdog scalers */
   clrwdt;
   WDTCON=0x17;	  /* 1:65k prescale, 2096.64ms time base */
   /* option reg: ps2,1,0 want 101 for 1:32 postscale, gives 67.1 sec watchdog 
      prime the interrupt pin: falling edge, please!
      PS2,PS0=1, PS1,INTEDG=0  */
   OPTION_REG=0b10111101;
   clrwdt;

   /* now switch to 8mhz intrc, we don't want to slog along at 31.25khz... */
   OSCCON|=0x70;
   while(!IOFS);		/* wait until freq stable */
   delay256ms();		/* and then a bit more as per errata  */
   clrwdt;

   /* setup environment: clock inthandler blinks the led in startup period */ 
   intblinky=1;		
   uptime=0; 
   seconds=0;

   /* setup timer1 and start clock interrupt */
   TMR1H=0x80;			/* so that it rolls over in one second */
   TMR1L=0;
   T1CON=0x0f;			/* async, ext clock,  */
   TMR1IE=1;
   PEIE=1;			/* tmr1 int counts as peripheral, also serial receive */
   INTE=1;			/* enable int pin interrupts */
   GIE=1;			/* ready for interrupts */

   /* porta/trisa: inputs except ra1=POWERBIT, which is enabled on demand only, 
      and ra2=ALIVEBIT which is mostly output */
   PORTA=0;
   TRISA&=~(1<<ALIVEBIT);
   PORTB=0;
   TRISB&=~(1<<LEDBIT); 	/* rb1 is output for the debug blinky */

   /* setup serial port for tx */
   SPBRG=25;			/* 19200bps with 8mhz intrc */
   BRGH=1;
   SYNC=0;
   TXEN=1;
   SPEN=1;

   /* setup adc operation */
   ANSEL=0x01; 			/* use RA0/AN0 for adc, all others digital */
   ADCS2=1;			/* ad clock t/16 */
   ADCS1=0; 
   ADCS0=1;
   /* adc is by default left-justified, ie. ADRESL has XX000000 and
      thus can be completely ignored for this application.
      default channel is ra0/an0. */

   /* what kind of startup? 
      power on reset -> do nothing special. 
      watchdog without status.ignwdt -> power down sim900 (means real 
      watchdog expiration)
      watchdog but status.ignwdt -> nothing special (means reset by ring)
   */
   if (NOT_POR && status.wdtexp && !status.ignwdt && isalive())
   {
      /* power down sim900 and delay until it's down for sure */
      killsim900(); 
      delayfive(1);
   }

   NOT_POR=1;	 /* need to set it: is only cleared on power on reset 
		    but left alone otherwise */
   status.ininit=0; /* we're not in the critical startup sim900 phase */
   status.ignwdt=0; /* and the next wdt is for real unless told otherwise */

   /* wait a tiny little bit, then power the sim900 up and configure it */
   delay256ms();
   clrwdt;
   initsim900();
   clrwdt;

   /* reset the counters, as now we're ready to work */
   uptime=0;
   seconds=0;
   TMR0=0;			/* tmr0 runs always, clear it now */
   intblinky=0;		/* inthandler stop blinking */

   const unsigned char s[]="AT+CIPSEND=9\r";
   unsigned char consecerrs=MAXCONSEC;
   while (1)
   {
      unsigned char cursec=seconds;
      /* wait until next second */
      while (seconds==cursec);
      clrwdt;

      /* now sample direction and save in dir[x] */
      ADON=1;
      /* AN546 says possibly up to 25us acquisition time required,
	 so we delay "a little" (very imprecise but should be sufficient) 
	 counting to 96 seems to work best. */
      for(i=0;i<96;++i);
      GO_DONE=1;		    /* start conversion */
      while(GO_DONE);		    /* and wait until its done */
      dir[seconds]=ADRESH;
      ADON=0;			/* and switch the adc off */
            
      /* check the status of the previous send. 3 seconds timeout 
	 for the response should be plenty */
      if (seconds == 2)
      {
	 RCIE=0;		/* disable serial reception and int */
	 CREN=0;
	 while (RCIF)		
	    i=RCREG;
	 if (status.okrespfound)
	 {
	    consecerrs=MAXCONSEC; /* reset error counter */
	 }
	 else
	 {
	    if (!--consecerrs)
	       reseteverything();
	 }
      }


      /* we have counted long enough, now send the stuff */
      if (!seconds)
      {
	 speed=TMR0;		/* capture counted speed pulses */
	 TMR0=0;

	 PORTB|=(1<<LEDBIT);	/* led on */

	 /* start the sim900 if not on */
	 if (initsim900())
	 {
	    TMR0=0;
	    seconds=0;		/* discard this set of readings 
				   and continue the main loop */
	    continue;
	 }

	 /* send cleanup \r's, then tx command, then data */
	 for (i=0;i<9;++i)
	 {
	    sertxchar('\r');
	 }
	 sertxstring(s);
	 delay256ms();	 	/* wait for the sim900 to 
				   react to the cipsend cmd and 
				   accept the data to be transmitted */

	 /* send data: uptime, speed, d5, d1..4, crc8 
	    (d5 is the newest reading) */
	 crc=0;
	 foo=(uptime>>8) & 0xff;
	 crc8(foo);
	 sertxchar(foo);

	 foo=uptime & 0xff;
	 crc8(foo);
	 sertxchar(foo);

	 crc8(speed);
	 sertxchar(speed);

	 for(i=0;i<sizeof(dir);++i)
	 {
	    foo=dir[i];
	    crc8(foo);
	    sertxchar(foo);
	 }
	 /* set up for serial reception to capture the status response */
	 respnextchar=0;
	 status.okrespfound=0;
	 CREN=0;		/* clear oerr if any present */
	 while (RCIF)		/* clear rcif and input fifo */
	    i=RCREG;
	 RCIE=1;
	 CREN=1;

	 sertxchar(crc);
	 /* wait 64ms before switching the led off */
	 __delay_ms(64);
	 PORTB&=~(1<<LEDBIT);		/* led off */
      }
   }
}

__interrupt void inthandler(void)
{
   /* timer 1/clock interrupt? */
   if (TMR1IF)
   {
      TMR1IF=0;
      TMR1H|=0x80;		/* prime timer to roll over in 1s */
      TMR1ON=0;			/* as per errata */
      TMR1CS=1;
      TMR1ON=1;
      ++seconds;

      if (intblinky)
	 PORTB^=(1<<LEDBIT);		/* 0.5hz blinky during startup */

      if (seconds==5)
      {
	 seconds=0;
	 ++uptime;
      }
   }
   /* serial receive interrupt? 
      note that rcreg is two-deep and rcif is cleared on read rcreg */
   while (RCIF)
   {
      if (RCREG == respstring[respnextchar])
      {
	 ++respnextchar;
	 if (!respstring[respnextchar]) /* reached the end of the search string */
	 {
	    status.okrespfound=1;
	    respnextchar=0;
	    CREN=0;		/* disable further receives */
	    RCIE=0;
	 }
      }
      else
      {
	 status.okrespfound=0;
	 respnextchar=0;
	 CREN=0;		/* disable further receives */
	 RCIE=0;		
      }
   }
   
   /* ring indication via interrupt pin? */
   if (INTF)
   {
      INTF=0;
      reseteverything();
   }
}

/* resets the sim900 and then the pic. doesn't return! */
void reseteverything()
{
   /* don't do anything while the sim900 is just being started */
   if (!status.ininit)	/* a real ring indicator */
   {
      PORTB|=(1<<LEDBIT);	/* led on */
      /* power down sim900 if it's up, leave it off otherwise */
      if (isalive())
	 killsim900();
      status.ignwdt=1; /* don't stop the sim900 after the reset 
			  as it's off already */
      /* set watchdog to expire in 8.4s to get a full reset */
      clrwdt;
      /* PS2,1,0 was 101 for 1:32, now 010 for 1:4 */
      OPTION_REG=0b10111010; 
      clrwdt;
      /* and twiddle your thumbs until then */
      while(1); 
   }
}

/* returns 1 if sim900 is up, 0 otherwise 
   doesn't check whether sim900 is initialized yet, just electrical 
   up or not */
unsigned char isalive()
{
   char x;

   /* make dtr an input for the test */
   TRISA|=(1<<ALIVEBIT);
      
   /* is dtr high? our setup of the sim900 doesn't use dtr for
      sleep purposes, so its state is ignored by the sim900 and left
      at 2.8V. if the sim900 isn't powered up, then dtr
      is at 0..1.5V. this is not useable for digital input, so we use
      the comparator and vref module. */
   
   CVRCON=0b10101011;         /* 0xab =  vref on, no pin output, 
				 11/24*vdd=~2.29v reference as c2+ */
   /* comparator on:
      ra3 on c1 (unused, don't care), ra2 on c2-
      inverted output please
      mode 010 (default is 111)
    */
   CMCON=0b00101010;		/* xx, c2inv, x, cis, rest is mode 010 */
   __delay_us(20);		/* delay 20us for settling */

   x=C2OUT;			/* 0 if dtr below vref = sim900 off, 
				   1 if dtr above vref = sim900 on */
      
   CVRCON=0;			/* vref off */
   CMCON=0x07;			/* mode 111, comparator off */

   PORTA&=~(1<<ALIVEBIT);	/* set this latch, read others */
   TRISA&=~(1<<ALIVEBIT);
   PORTA&=~(1<<ALIVEBIT);	/* make sure dtr stays pulled low */
   
   return x;
}


/* power the sim900 down. assumes that it's alive (doesn't check again).
   does not delay afterwards! */
void killsim900()
{
   char x;
   
   PORTA&=~(1<<POWERBIT);	/* prep to drive low: sets this latch, 
				   others are read */
   TRISA&=~(1<<POWERBIT);
   PORTA&=~(1<<POWERBIT);	/* drive low */
   for (x=0;x<5;++x)
   {
      delay256ms();		/* 128ms x 10 = 1.28s */
   }
   TRISA|=(1<<POWERBIT);	/* back to input */
   return;
}
   
/* check the sim900's state, if off power it up, wait until it's 
   alive and initialize it. 
   led is set to on while doing work.
   returns 1 if it did power up the sim900, 0 if it was up already 
   
   takes about 43s with all the delays, so clrwdt before! */
unsigned char initsim900()
{
   char x;

   if (isalive())
      return 0;

   status.ininit=1; /* critical phase: don't reset while it's being powered up
		       as that would falsely indicate a started and ready sim900
		       after the reset completes */
   intblinky=0;
   PORTB|=(1<<LEDBIT);		/* led on */
   /* make powerbit an output, pull it low for 1.2s, then let go */
   PORTA&=~(1<<POWERBIT); /* prep to drive low: this latch is set, 
			     others read */
   TRISA&=~(1<<POWERBIT);
   PORTA&=~(1<<POWERBIT);	/* drive low */

   for (x=0;x<6;++x)
   {
      delay256ms();		/* 128ms x 10 = 1.28s */
   }
   TRISA|=(1<<POWERBIT);	/* back to input */
   status.ininit=0;

   intblinky=1;
   PORTB&=~(1<<LEDBIT);		/* led off */
   /* wait 30s, now the sim900 should be ready to talk to us */
   delayfive(6);
   intblinky=0;
   PORTB|=(1<<LEDBIT);		/* led on */

   /* send the various config & initializer strings: */
   /* auto-baud, slightly slowly just in case */
   sertxchar('A');
   delay1ms();
   sertxchar('T');
   delay1ms(1);
   sertxchar('\r');
   delay256ms();

   /* 19200bps fixed, rts/cts flow control, don't ignore DTR!, 
      remove all sms, disable URC-signals-RI, no echo, and save settings */
   const unsigned char s1[]="AT+IPR=19200;+IFC=2,2;+CMGDA=6;+CFGRI=0;E0&D1&W\r";
   sertxstring(s1);
   for(x=0;x<8;++x) 		
      delay256ms();		/* 2.048s */
   
   /* send the 'do connect string' */
   const unsigned char s2[]="AT+CIPCSGP=1,\""ISP"\";+CIPSTART=\"udp\",\""SERVER"\",\""PORT"\"\r"; 
   sertxstring(s2);

   intblinky=1;
   PORTB&=~(1<<LEDBIT);		/* led off */
   delayfive(2);		/* make sure to wait until the conn is up */
   intblinky=0;
   PORTB&=~(1<<LEDBIT);		/* led off */

   return 1;
}

/* wait x times 5 seconds 
   error is below one second */
void delayfive(unsigned char fivers)
{
   unsigned char nowsecs=seconds,x;
   
   for(x=fivers;x>0;--x)
   {
      /* wait until next second, then wait until the wanted 
	 second arrives again = 5s (minus fractional sec) */
      while(seconds==nowsecs);
      while(seconds!=nowsecs);
   }
}

/* send one byte with flow control *and* a little bit of extra delay,
   just to be on the safe side */
void sertxchar(char what)
{
   /* wait until cts=low and txif is high */
   while (PORTB & (1<<CTSBIT) || !TXIF);
   TXREG=what;
   __delay_us(50);
   return;
}

/* send a string (null-term) with flow control *and* extra delay,
 */
void sertxstring(const unsigned char *what)
{
   unsigned char i,j;

   for(i=0;j=what[i];++i)
   {
      sertxchar(j);
   }
   return;
}   
   

/* updates crc var factoring in data, returns crc value */
unsigned char crc8(unsigned char input)
{
   crc^=input;
#asm
      clrw

      btfsc	_crc,0
      xorlw	0x5e

      btfsc	_crc,1
      xorlw	0xbc
      
      btfsc	_crc,2
      xorlw	0x61
      
      btfsc	_crc,3
      xorlw	0xc2
      
      btfsc	_crc,4
      xorlw	0x9d

      btfsc	_crc,5
      xorlw	0x23
      
      btfsc	_crc,6
      xorlw	0x46
      
      btfsc	_crc,7
      xorlw	0x8c
      
      movwf	_crc
#endasm
   return crc;
}

