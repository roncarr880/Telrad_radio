/*
   K1URC
     Israeli 3X KVN filter radio
         MC1496 mixers and MC1350 IF amp
     Chipkit uC32
     SI5351 vfo bfo
     128 by 32 OLED display.  Use two pages as 1305 controller has enough ram for a 128 by 64 display
     Band select via I/O expanders and relay drivers.  Bands are 80,60,40,30,20,15.  Could be changed
     with a different plug in bandpass filter.
     QRP-LABS 5 watt PA. Raised cosine shaped cw waveform
        Changes: Left off signal leakage gate, Added negative feedback, output transfomer as 2:1 instead of 3:1 .
     
 
*/   

/*   things to implement or consider
  !!! fix the K3 emulation for the 100x frequency changes
  AM detector fix and AGC
  S  meter
  SWR bridge wiring to A/D converters
  PA temperature wiring
  Line In and Line out wiring ( need a 600 ohm transformer for line in )
  Rx Preamp?
  Should we forget about XIT?  Never used it.  Maybe use in place of 2 vfo's.  Or would RIT handle all splits.
    Any carrier mode will need an offset to get through the filters.  Put this on 1st mixer like with cw_offset.
    That would account for any dual vfo's like used for wspr on the rebel. wspr tx offset 1400 to 1600.
  FFT audio display.
  CW decoder
  WSPR standalone transmit.
  Digital audio
    CW filters 
    Auto notch
    Denoise
 !!! not sure swapping the PLL assignments for TX is really buying anything other than confusion in how it works
 !!! it seems we have to load a new bfo and vfo anyway due to rit,xit,tone control. And reset the PLL's.
*/


//  starting addresses of phase lock loop registers  A is bfo,  B is vfo
#define PLLA 26
#define PLLB 34

#define CW 0       // modes
#define LSB 1
#define USB 2
#define AM  3
#define MEM 4
#define WSPR 5

#define RX 0           // vfo calc
#define TX 1

#define FREQ 0         // encoder users
#define MENU_E 1
#define MEM_TUNE 2
// #define MESSAGE 3

// addresses of things on the I2C bus
#define D2A_AUDIO  0x63  //  audio output address
#define D2A_POWER  0x62  //  power out / IF gain control on i2c bus
#define SI5351     0x60  //  I2c address of clock chip
#define IO_EXP0    0x20  //  bandpass filters plus a couple of extra relay drivers
#define IO_EXP1    0x21  //  tx relay drivers, audio mux mute and 1 extra unbuffered pin

#define OLAT  0x0a       // internal LAT register for the IO expanders
#define IODIR  0         // register defines pin as input or output.  We will set up all outputs on the expanders.
                         // Only have one potential input pin as all the others have a FET buffer.

//  the order of the signals on J7 is gnd,am,ssb,d/a
//  io_exp1 bit definitions
#define AM_SEL   64      // audio mux mask values for the io expander #1
#define SSB_SEL  128 
#define DIG_SEL  32      // selects D2A_AUDIO device as a sound source.  Digital audio or sidetone.
                         // !!! we need to select a resistor value to complete this circuit
#define AM_MUTE  8       // extra to prevent strong signal bleed through
#define CWU      16      // Unbalances the 1st mixer to generate a CW carrier.  The only unbuffered output.
#define TR_RELAY  1
#define AB_DRIVER 2      // EMRFD class AB driver circuit enable

#define SCLK  3          // for the PA D/A using a 74HC595.  QRP-LABS kit.
#define RCLK  4
#define SER   5

// #include <Wire.h>  not used because the library routines block on start and stop functions
#include "si5351_init_20m.h"     // data for initial load of all the SI5351 registers 
#include "sine_cosine.h"

// Nokia LCD library converted to an OLED library to drive the OLED display

//      Ground           OLED pin 1
//      3.3 volts        OLED pin 2
//      SCK  - Pin 11    OLED PIN 7
//      MOSI - Pin 10    OLED PIN 8
//      DC   - Pin 12    OLED PIN 4
//      RST  - Pin 8     OLED PIN 16
//      CS   - Pin 9     OLED PIN 15
//

#include <OLED1305_Basic.h>

OLED1305 OLED(11,10,12,8,9);  

extern unsigned char SmallFont[];
extern unsigned char MediumNumbers[];
// extern unsigned char BigNumbers[];  not in use for this program

const uint64_t clock_freq = 2499960000;   // frequencies are times 100 for 2 decimal places
uint64_t temp_correction;   // clock_freq correction
int drift_defeat;       // no temp correction if it looks like the temp sensor is not working correctly

// frequency determining variables
uint64_t vfo,bfo;
uint32_t cw_offset = 80000;
uint32_t wspr_offset = 153800;
// int band = 4;    // 20 meters is loaded from the clockbuilder file and must be the default for startup
int band = 2;       // maybe not if we reset the PLL's with a call to band_change2
const int bfo_divider = 70;   // 70 in the clock builder file. don't change.

// raised cosine shaping with QRP-LABS PA.
int tx_indx, tx_cnt;

/*
  Bandpass cabling from IO Expander to QRP-Labs Arduino shield Analog connector.

Relay Pos:    1      2      3      4      5      0
Band:        15     20     30     40     60     80
Connector:   A0     A1     A2     A4     A3     A5
Mask          1      2      4     16      8     32

 Notice A4 and A3 are swapped from expected order due to existing wiring on the qrp-labs arduino shield.
 A3 is wired to position 5.
 We will plug the filters in the expected order and use the swapped masks as shown above.
 
 Wiring needed on the qrp-labs arduino shield:  A1 to 10, A2 to 11, A4 to 12, A5 to 7
     A0 and A3 are already connected to a relay.   
*/
// set up a band structure that has all we need
// freq, divider, rit, xit, mode, step, pin, sideband, power
struct BAND {
  uint64_t freq;     
  int divider;    // si5351 output divider( always an even number for this application and fixed per band )
  int rit;        // value of rit offset
  int rit_state;  // is rit active on this band
  int mode;       // cw usb lsb
  int bandpass;   // bit mask for front end bandpass filter ( io expander )
  int power;      // value of 0 to 100 that will map into a 12 bit d/a value for MC1350 gain
 // int tone_offset;  // tone control.  Moves BFO for different filter passband.
  int xit;        // xit offset
  int xit_state;       // 0-not in use   1-inuse but encoder changes freq   2-inuse and encoder changes offset
};                     // xit not implemented, may remove any mention of it
                       // and use things like cw_offset, wspr_offset etc to get desired split for some modes
                       // that will work within a 200 to 3200 bandwidth

// band stacking registers
// divider an even number close to 750 / ( freq + IF )
struct BAND band_info[6] = {
  { 392800000,58,0,0,LSB,32,40,0,0 },
  { 533050000,52,0,0,USB,8,70,0,0 },
  { 718500000,46,0,0,LSB,16,50,0,0 },
  {1011000000,40,0,0,CW,4,50,0,0 },
  {1407600000,32,0,0,USB,2,70,0,0 },
  {2126000000,24,0,0,USB,1,90,0,0 }
  /* {28500000,20,0,0,USB,32,0,0 } */
  };

struct MEMORY {
  char name[15];
  uint64_t freq;
  int band;  // 0 - 80m ,  1 - 60m, 2 - 40m, 3 - 30m, 4 - 20m, 5 - 15m or 10m or whatever bandpass we install
  int mode;  // cw usb lsb am
};

#define NOMEM 28
const struct MEMORY memory[NOMEM] = {
   { "SeaGull Net", 394000000, 0, LSB },
   { "WSPR 80 rx",   359260000, 0, USB },
   { "WSPR 60 RX",   528720000, 1, USB },
   { "WSPR 40 rx",   703860000, 2, USB },
   { "WSPR 30 rx",  1013870000, 3, USB },
   { "WSPR 20 rx",  1409560000, 4, USB },
   { "WSPR 15 rx",  2109460000, 5, USB },
   { "W1AW DIGI",   709500000-150000, 2, USB },  
   { "W1AW Code"   , 704750000-80000, 2, CW },   
   { "60m Chan 1",   533050000, 1, USB }, 
   { "60m Chan 2",   534650000, 1, USB },
   { "60m Chan 3",   535700000, 1, USB },
   { "60m Chan 4",   537150000, 1, USB },
   { "60m Chan 5",   540350000, 1, USB },
   { "Boston WEFAX", 634050000-190000, 2, USB },
   { "Air Volmet",   660400000, 2, USB },
   { "Air NY E",     662800000, 2, USB },
   { "Air Carib A",  657700000, 2, USB },
   { "Air Carib B",  658600000, 2, USB },
   { "Air LDOC",     664000000, 2, USB },
   { "Air NY E",    1335400000, 4, USB },
   { "Air Carib B", 1329700000, 4, USB },
   { "Air LDOC",    1333000000, 4, USB },
   { "CHU Canada",   785000000, 2, USB },     // chu is usb with carrier re-inserted  
   { "WWV",          500000000, 1, USB },   
   { "WWV",         1000000000, 3, USB },   
   { "WWV",         1500000000, 4, USB },
   { "WWV",         2000000000, 5, USB }   
}; 

int mem;    // current memory channel in use

struct BAND_LIM {
  unsigned long start_;
  unsigned long end_;
};

//  amateur advanced class band limits
const struct BAND_LIM band_lim[10] = {
  {  3525000L,  3600000L },
  {  3700000L,  4000000L },
  {  5330400L,  5403600L },   // careful transmitting here
  {  7025000L,  7300000L },
  { 10100000L, 10150000L },
  { 14025000L, 14150000L },
  { 14175000L, 14350000L },
  { 21025000L, 21200000L },
  { 21225000L, 21450000L },
  { 28000000L, 29700000L }
};

#define STQUESIZE 64
unsigned char stg_buf[STQUESIZE];  /* stage buffer to avoid blocking on serial writes */
int stg_in = 0;
int stg_out = 0;
  
  
// current working values for freq control
// load these from the band stacking or from the memory tuning data
uint64_t freq;
int rit, rit_state;
int xit, xit_state;
int mode, stp;
int tone_offset;


int tx_inhibit = 0;
int freq_mod;            // zero out digits smaller than the freq step size - one time adjustment when step size has been changed
int encoder_user;        // process that is using the encoder
int wpm = 12;            // keying speed
int transmitting = 0;    // flag for tx on or tx off
int tx_holdoff;
int audio_flag;
long audio;

#define LOCK 0x4      // make up some names for the toggle switches
#define PTT  0x8      // their use may be redefined from the name
#define MULTI 0x40
#define DECODE  0x20
#define PAGE    0x10
int toggles;    //  wired on port E



// polling I2C implementation because wire library blocks
#define I2BUFSIZE 128
int i2buf[I2BUFSIZE];
int i2in, i2out;

// holder variables for analog read values.  All analog reads must be in the dsp_core function
// as the analog read function is not re-entrant
int osc_temp, pa_temp, multi_pot;

uint16_t expander1;    // a copy of what has been loaded into io_expander1 rx upper byte, tx lower byte
int multi1 = 0;    // multi POT functions assigned via menu.  RIT is usually multi0 but doesn't have to be
int multi2 = 2;    // defaults are RIT off and Tx power.
unsigned long mult_timeout;     // time out the display so it looks cleaner

int semi_breakin_timer;
int ifgain = 100;          // 0 to 100 mapped to inverse 12 bits for D/A

int wspr_duty = 2;
int wspr_tx_enable;

/*************************************************************************************************/

void setup(){
int i;

  pinMode(SCLK,OUTPUT);
  pinMode(RCLK,OUTPUT);
  pinMode(SER,OUTPUT);
  digitalWrite(RCLK,LOW);
  tx_load( 0 );        // remove qrp-labs PA power
  
  OLED.InitLCD();
  OLED.clrScr(0);  // clear pages 0 and 1
  OLED.clrScr(1);
  OLED.setFont(SmallFont);
  
  // encoder on PORT RE bits 0 and 1
  pinMode( 26,INPUT);
  pinMode( 27,INPUT);
  // Toggle switches on PORT RE 2 - 6
  pinMode( 28,INPUT);    // input_pullup does not work, at least on port E
  pinMode( 29,INPUT);
  pinMode( 30,INPUT);
  pinMode( 31,INPUT);
  pinMode( 32,INPUT);
 
  pinMode( 45, INPUT );
  pinMode( 46, INPUT );  // ?? needed for i2c
  pinMode( 13, OUTPUT ); // front panel led
  
  
   pinMode( 2, OUTPUT );   // negative charge pump
    // pwm pins 3,5,6,9,10.  9 and 10 are wired to the LCD leaving 3,5,6
    // 3 and 5 go to the PA leaving 6
    // charge pump is using the core timer at 16k rate instead of PWM
  attachCoreTimerService( dsp_core );        // audio processing and -12 volt charge pump
  //delay(1000);    // wait for -12 volts to charge up

  // check if in tx mode and hang until toggled
  if( PORTE & PTT ){
    OLED.print("Turn off PTT",CENTER,0);
    OLED.print("Turn off PTT",CENTER,4*8);
    while( PORTE & PTT );
    OLED.clrScr(0);  OLED.clrScr(1);
  }
  
  // start up our local I2C routines
  I2C1CONSET = 0x8000;      // I2C on bit
  I2C1BRG = 90;             // 400 khz
  i2stop();                 // clear any extraneous info that devices may have detected on powerup
  
  // load up the results of clock builder
  // si5351 outputs are on 23.060  and 8.999
  si5351_init();
  
  // load the working freq values
  freq = band_info[band].freq;
  rit = band_info[band].rit;
  xit = band_info[band].xit;
  rit_state = band_info[band].rit_state;
  xit_state = band_info[band].xit_state;
  mode = band_info[band].mode;
  //tone_offset = band_info[band].tone_offset;
  
  // set the toggles variable to the inverse of what the toggle switches are
  //  to force all switches to be processed the first pass through loop()
  toggles = (PORTE & 0b1111100) ^ 0b1111100;
  // the step size toggle is dual use with decode, so need to process that one here
  // toggles ^= DECODE;   step_change(); 
  stp = 100;  // or initialize the variable and let loop take care of it if we are in the main screen
              // else if on screen 2, step will not be changed == normal operation  

  //  power out and IF gain default is zero == max gain of the MC1350
  //  power &= 0xfff;  // mask to valid number of bits
  // i2cd( D2A_POWER, power >> 8, power & 0xff );
  i2cd( D2A_POWER, 0, 0 ); 
  i2cd( D2A_AUDIO, 8, 0x00 );  // half scale audio out 0x800 for 12 bit d/a converter
  i2flush();
  
//  vfo = freq + bfo;
//  si_pll_x(PLLB, vfo, band_info[band].divider);
//  freq_display(freq);
//  status_display();   // top line
 
 // set up the I/O expanders and exercise the relays
  i2cd( IO_EXP0, IODIR, 0 );   // all outputs
  for( i = 128; i > 0;  i >>= 1 ){
    i2cd( IO_EXP0, OLAT, i );
   // i2cd( IO_EXP1, OLAT, i );   not switched to avoid transmitting
    i2flush();                
    delay( 200 );
  }
  i2cd( IO_EXP1, IODIR, 0 );   // all outputs
//  load_expander();    // load the i/o expanders with the correct initial values

  // change to power up default conditions
  bfo = 900000000;  
  si_pll_x(PLLA,bfo,bfo_divider);
  band_change2();       // sets vfo, divider, inits PLL's, loads the expander, displays freq and status.
  i2flush();
  set_ifgain();
  
  Serial.begin(38400);   // K3 emulation 
  
}


void loop(){
int temp, temp2;
static int dbounce;
static int dcount;
static time_t m;
static int slow_encode;  // slow down the encoder for menu selection
static int msec;

  while( m == millis()){           // features needing fast code here
     
     i2poll();       // keep the i2c bus running
     
     if( audio_flag ){
     //   temp code for now - simple audio pass through
      //  audio *= 2;         // gain
      //  audio += 2048;         // 12 bits output D/A mid point
       // i2cd( D2A_AUDIO, ((audio >> 8 )& 0xff), audio & 0xff );
        audio_flag = 0;

     }
  
  }
  
  m = millis();
  if( mode == WSPR ) wspr_timer(m);
  ++msec;  
  if( msec == 2000 ){
    msec = 0;
  // temperature readings
    if( transmitting ) pa_temp_display(pa_temp);
    else{ 
       osc_temp_display(osc_temp);
       drift_compensation();  // can't compensate during tx as dividers are swapped for si5351
                              // !!! revisit this as we have changed how the bfo and vfo are swapped for tx
    }
  }
  
// test to see if this makes noise  hangs when not present??
// i2cd(D2A_AUDIO,0x55,0xAA);

  // encoder users
  temp = encoder(); 
  if( encoder_user != FREQ ){
     slow_encode += temp;
     temp = 0;
     if( slow_encode > 10 ) temp = 1, slow_encode = 0;
     if( slow_encode < -10 ) temp = -1, slow_encode = 0;
  } 
  if( temp ){
     switch( encoder_user ){
        case FREQ: if( (toggles & LOCK) == 0 ) freq_user(temp); break;
        case MEM_TUNE: mem_tune_user(temp);  break; 
        case MENU_E: top_menu(temp);  break;
     }
  }
 
  temp = PORTE & 0b1111100;   // read switches
  if( encoder_user == MENU_E ) temp |= PAGE;      // force menu on page 1 to display
  temp2 = temp ^ toggles;     // detect changes
  if( encoder_user == MENU_E ) temp2 &= LOCK+PAGE;     // ignore other switches when in the menus
  if( temp2 ){
    if( dbounce == temp2 ) ++dcount;
    else dcount = 0;
    dbounce = temp2;
    if( dcount >= 20 ){    // n millisec debounce
       toggles ^= temp2;
       toggle_change(temp2);
    }
  }
  else dcount = 0;
  
  check_band_limits();
  tune_band_change();   // see if tuning around general coverage and a band filter change needed
  
  /* 1 char per ms or 1000 cps or 10000 baud, sort of half duplex HRD K3 emulation */
  if( un_stage() == 0 ) radio_control();     /* nothing to send, get next command */ 
  
  if( mode == CW ) paddles();
  if( semi_breakin_timer ){
    if( --semi_breakin_timer == 0 ) tx_off(0);
  }
  
  multi_dispatch();     // process the multi function analog pot changes if any
  
}

void multi_dispatch(){
int f;
static int last_val;
static int last_toggle;
int linear;

// it seems my multi pot is audio taper instead of linear
 //OLED.printNumI(multi_pot,6*0,5*8,4,' ');    // debug
 
   if( multi_pot < 75 ) linear = map(multi_pot,0,75,0,250);
   if( multi_pot >= 75 && multi_pot < 200 ) linear = map(multi_pot,75,200,250,500);
   if( multi_pot >= 200 && multi_pot < 500 ) linear = map(multi_pot,200,500,500,750);
   if( multi_pot >= 500 ) linear = map(multi_pot,500,1023,750,1023);
 //OLED.printNumI(linear,6*0,6*8,4,' ');       // debug  
   
   f = toggles & MULTI;
   if( f != last_toggle ) mult_timeout = 0, last_toggle = f;
   if( multi_pot != last_val ) mult_timeout = 0, last_val = multi_pot;
   
   if( mult_timeout == 0 ){    // dispatch a change in something
      f = (f == 0) ? multi1 : multi2;
      switch(f){
        case 0:  mult_timeout = 19999;   break;   // null function.  force a clear line on display
        case 1:  rit_adjust(linear);   break;      // linear response
        case 2:  set_tx_power(linear); break;
        case 3:  set_tone(linear);     break;
        case 4:  adjust_ifgain(linear);   break;
      }
   }
   
   if( ++mult_timeout == 20000 ){               // clear the display area for multi message
      //OLED.print("          ",6*11,2*8);      // just to have a cleaner looking display
      OLED.clrRow(2,6*11);
   }
   
  
}

void rit_adjust(int val){
int rit2;

   rit2 = val - 500;
   if( (rit - rit2) == 1 ) return;   // ignore noise on rit pot
   if( (rit - rit2) == -1) return;
   
   rit = rit2;
   vfo = calc_vfo(RX); 
   si_pll_x(PLLB,vfo,band_info[band].divider);
   OLED.print("  RIT ",6*11,2*8);
   OLED.printNumI( rit ,6*17,2*8,4,' ');
  
}

// adjust rit state on menu changes(0) and on band changes(1)
void rit_onoff( int fun ){

   if( fun == 0 ){     // multi menu change, alter rit_state
      if( multi1 == 1 || multi2 == 1 ) rit_state = 1;
      else rit_state = 0;
   }
   else{               // band change, alter multi
      if( rit_state == 1 ){            // find a multi position to put rit on
        if( multi1 == 0 ) multi1 = 1;
        else if( multi2 == 0 ) multi2 = 1;
        else multi1 = 1;
      }
      else{            // remove rit from multi
        if( multi1 == 1 ) multi1 = 0;
        if( multi2 == 1 ) multi2 = 0;
      }
   }
   // finally turn the led on or off on this change
   if( rit_state ) digitalWrite(13,HIGH);
   else digitalWrite(13,LOW);
}

// set power and write the display
void set_tx_power(int val){     // called from multi dispatch.  
int pwr;
String nogo;

   nogo = " ";
   pwr = map(val, 0, 1023, 0, 100 );
   // re-center the knob to the actual value without changing anything
   if( pwr > (band_info[band].power + 5) ) pwr = band_info[band].power, nogo = ">";
   else if( pwr < (band_info[band].power - 5) ) pwr = band_info[band].power, nogo = "<";
   band_info[band].power = pwr;
   OLED.print("Tx Drv",6*11,2*8);
   OLED.print(nogo,6*17,2*8);
   OLED.printNumI( pwr ,6*18,2*8,3,' ');
   if( transmitting ) set_power();          // effect the power change now if tx is active
                                            // may want to check the swr or reflected pwr first
}

void set_tone( int val ){    // move the bfo around to change the audio tone
int t_off;
String nogo;
    
    nogo = " ";
    t_off = map(val,0,1023,-999,999);            // can adjust tone range here
    if( t_off > (tone_offset + 50 )) t_off = tone_offset, nogo = ">";
    if( t_off < (tone_offset - 50 )) t_off = tone_offset, nogo = "<";
    tone_offset = t_off;
    OLED.print("Tone ",6*11,2*8);
    OLED.print(nogo,6*16,2*8);
    OLED.printNumI(tone_offset,6*17,2*8,4,' ');
  
    t_off = (mode == USB)? tone_offset: -tone_offset;
    bfo = 900000000 + 100*t_off;
    vfo = calc_vfo(RX);                           // as we move the bfo, the vfo also moves to stay on freq
    si_pll_x( PLLB,vfo,band_info[band].divider);
    si_pll_x( PLLA,bfo,bfo_divider);
  
}

void adjust_ifgain(int val){
int gain;
String nogo;

   nogo = " ";
   gain = map(val, 0, 1023, 0, 100 );
   // re-center the knob to the actual value without changing anything
   if( gain > ifgain + 5 ) gain = ifgain, nogo = ">";
   else if( gain < ifgain - 5) gain = ifgain, nogo = "<";
   ifgain = gain;
   OLED.print("IF Vol",6*11,2*8);
   OLED.print(nogo,6*17,2*8);
   OLED.printNumI( ifgain ,6*18,2*8,3,' ');
   set_ifgain();
}

void load_expander(){   // load the io_expanders
int sideband_select;
int audio_select;

  sideband_select = ( mode == LSB ) ? 64 : 0 ;
  audio_select = (mode == AM) ? AM_SEL : SSB_SEL + AM_MUTE;

  i2cd( IO_EXP0, OLAT, band_info[band].bandpass | sideband_select );
  i2cd( IO_EXP1, OLAT, audio_select );        // also will unkey TX
  expander1 = audio_select * 256;             // save a global copy for tx/rx switching
  
}

void mem_tune_user( int temp ){
  
  mem += temp;
  if( mem >= NOMEM ) mem = 0;
  if( mem < 0 ) mem = NOMEM - 1;
 
  freq = memory[mem].freq;
  mode = memory[mem].mode;
  band = memory[mem].band;
  
  band_change2();
  
}

// process anything that is needed immediately when a toggle changes 
void toggle_change( int val ){ 
  
   if( val & LOCK ){
     if( encoder_user != MENU_E ) menu_start_end(toggles & LOCK);
     else menu_select();
   }
   if( val & PAGE ){
       if( toggles & PAGE ) OLED.viewPage( 1 );
       else OLED.viewPage( 0 );
    }
   if( val & DECODE ){   // dual use.  also step size when not on page 1
      if( (toggles & PAGE) == 0 ) step_change();
   }
   if( val & PTT ){
      if( (toggles & PTT ) ) tx_on();
      else tx_off(1);
   }
}

void menu_start_end(int val){
static int save_user;

  OLED.clrScr(1);
  if( val ){
     // print where we are in the menu's
     top_menu(0);
     if( encoder_user != MEM_TUNE ) save_user = encoder_user;
     encoder_user = MENU_E;
  }
  else{   // note: there is an extraneous call to here when toggle to zero the lock/menu
    encoder_user = ( mode == MEM ) ? MEM_TUNE : save_user;
    status_display();
    if( encoder_user == MEM_TUNE ) mem_tune_user( 0 );
  } 
}

void menu_select(){
int t;  
  // if no change then menu end
  t = top_menu(2);
  if( t == 0 ) menu_start_end(0);

}

void freq_user( int temp ){

  if( freq_mod ){                    // one time adjustment when step size change
     if( temp == -1 ) freq_mod = -freq_mod;
     else freq_mod = stp - freq_mod;
  }
  else freq_mod = temp * stp;

  freq += 100*freq_mod;
  if( xit_state == 1 ) xit -= freq_mod;  // xmit freq fixed and tuning around the receive freq
  freq_mod = 0;
  vfo = calc_vfo(RX); 
  si_pll_x(PLLB,vfo,band_info[band].divider);
  freq_display(freq);  
}

void drift_compensation(){
static int ave_val[16];
int val;
long current;
static int i;
int j;

//   oversampling not working well.  Perhaps needs some dither as readings are very steady.
//   there is about a 4.5 hz drift per ADC step whcih makes the resolution of this compensation
//   somewhat crude. 

   current = temp_correction;

   ave_val[i++] = osc_temp;  // average to prevent max min values from moving the compensation
   i &= 15;
   val= 0;
   for(j = 0; j < 16; ++j ) val += ave_val[j];
   val >>= 4;
     
   val = map(val,463*8,473*8,0,45);    // *8 == oversampled for 3 more bits
   if( val > current+1) ++temp_correction;   // hysterisis +- 1 count
   if( val < current-1) --temp_correction;   // move slowly in desired direction
       //temp_correction = current = 0; //defeat this algorithm for testing
   if( drift_defeat ) temp_correction = 0;
   if( temp_correction != current ){    // make the change
      si_pll_x( PLLB,vfo,band_info[band].divider);
      si_pll_x( PLLA,bfo,bfo_divider);
      //i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset, makes noise
   } 
}

//void rit_xit_user( int temp ){
  // save for coding example
//  if( rit_state ) rit += 10 * temp;
//  else xit += 10 * temp;
//  vfo = calc_vfo(RX); 
//  si_pll_x(PLLB,vfo,band_info[band].divider);
//  rit_xit_display();   
//}




  //  !!!!! testing carrier supression, reduce the drive on the bfo
  // i2cd( SI5351, 18, ( attn ) ? 0x4c : 0x4f );

uint64_t calc_vfo(int rx_tx){
// where the vfo should be depends upon what features are in use
uint64_t val;

    val = ( rx_tx == RX )? bfo : 900000000;
    val += freq;                          // high side vfo
    if( rx_tx == RX ){
      if( rit_state ) val += 100*rit;
      if( xit_state == 2 ) val += 100*xit;     // receiving on xit transmit freq
    }
    else{                                 // setting up for transmit
      if( xit_state ) val += 100*xit;     // transmitting on xit transmit freq      
    }
    
    return val;
}

void step_change(){

  if( toggles & DECODE )  stp = ( mode == CW ) ? 100 : 500;
  else stp = ( mode == CW ) ? 10: 100;
  freq_mod = freq % stp;  
  
}
//void step_change( int val ){  
//int t;

//   t = stp;
//   if( val == TAP ) t *= 10;
//   else t /= 10;
//   if( t == 0 ) t = 1000;
//   if( t > 1000) t = 1;
//   if( mode == CW && t == 1000 ) t = 100;   // tune cw slower.  Useful or a pain for big qsy
//   stp = t;
//   freq_mod = freq % t;   // one time adjustment next time encoder is used to change freq
//   status_display();
//}


//void mode_change1( int val ){   // mode change up down by one TAP or DTAP ( changed to use just TAP in the calling code )

//  if( val == TAP ) ++mode;
//  else --mode;
//  if( mode < 0 ) mode += 3;
//  if( mode > 2 ) mode -= 3;
//  bfo = bfo_freq[mode]; 
//  si_pll_x(PLLA,bfo,bfo_divider);
//  vfo = calc_vfo(RX);                // recalc vfo so stay at same frequency
//  si_pll_x(PLLB,vfo,band_info[band].divider);

//  digitalWrite(CW_PIN,(mode == CW)?HIGH:LOW);
  
//  status_display();  
//}

void band_change1( int val ){   // band change

   // save the current working values
   if( encoder_user != MEM_TUNE ){
      band_info[band].freq = freq;
      band_info[band].rit = rit;
      band_info[band].xit = xit;
      band_info[band].rit_state = rit_state;
      band_info[band].xit_state = xit_state;
  //    band_info[band].stp = stp;
      band_info[band].mode = mode;
     // band_info[band].tone_offset = tone_offset;
   }
   band = val;
   
   // load the new working values for new band
   freq = band_info[band].freq;
   rit = band_info[band].rit;
   xit = band_info[band].xit;
   rit_state = band_info[band].rit_state;
   xit_state = band_info[band].xit_state;
  // stp = band_info[band].stp;
   mode = band_info[band].mode;
  // tone_offset = band_info[band].tone_offset;
   rit_onoff(1);
   
  // encoder_user = FREQ;          // escape memory tuning if that is where we were
   band_change2();

}

void band_change2(){   // band change
int t_off;

   load_expander();   // write to io expanders
   t_off = (mode == USB)? tone_offset: -tone_offset;
   bfo = 900000000 + 100*t_off;
   vfo = calc_vfo(RX);
   si_pll_x(PLLA,bfo,bfo_divider);
   si_pll_x(PLLB, vfo ,band_info[band].divider);
   
      // set the new divider and init pll's
   si_load_divider( band_info[band].divider,0,1 );
 //  digitalWrite(CW_PIN,(mode == CW)?HIGH:LOW);
   
   status_display();
   freq_display(freq);
   step_change();       // ????? should we even be saving the stp in the band stacking now that we have toggles and not pushbuttons
                        // may still need this call because the step is mode dependant and the mode may have changed
}

void status_display(){
int x;

  
  if( encoder_user == MEM_TUNE ){
    status_display2();
  }
  else OLED.clrRow(3,0,14*6);
  
  if( tx_inhibit ) OLED.invertText(true);     // if out of band limits the mode is inverse video
  switch (mode){
     case CW:   OLED.print(" CW  ",16*6,0); break;
     case LSB:  OLED.print(" LSB ",16*6,0); break;
     case USB:  OLED.print(" USB ",16*6,0); break;
     case AM:   OLED.print(" AM  ",16*6,0); break;
     case WSPR: OLED.print(" WSPR",16*6,0); break;
  }
  OLED.invertText(false);
  
  // is the vfo locked
  if( (toggles & LOCK) && (encoder_user == FREQ) ) OLED.print("LK",11*6,1*8);   // 18*6
  else OLED.print("  ",11*6,1*8);
  
  if( mode != WSPR ) OLED.clrRow(1,6*18);   // clear the time counter area
  
}

void status_display2(){
 // display the memory tune text on the 4th line
 OLED.clrRow(3,0,14*6);
 OLED.print(memory[mem].name,0,8*3);
}


void freq_display( uint64_t val ){
int x;

//  freq is always on page 0 of the display in dedicated area
//  and display should always be clean
      val/= 100;
      OLED.setFont(MediumNumbers);
      x=  0;    // 4 *12;
      OLED.printNumI(val/1000,x,0,5,'/');  //prefix 5 character freq with spaces if less than 10000
      OLED.setFont(SmallFont);               // space is not part of the medium font, so using a period
      OLED.printNumI((val)%1000,5*12,0,3,'0');
      
      // also print on page 1 in small numbers, line 4, top line
      OLED.printNumI(val/1000,RIGHT,8*4,6,' ');  // prefix one or two spaces
}

void osc_temp_display( int val ){
  
  //OLED.clrRow(4,0,10*6);
  //OLED.printNumI(val/8,LEFT,4*8);   // debug
  //OLED.printNumI(temp_correction,5*6,4*8);
  
   val >>= 3;    // remove oversampled bits
   val *= 6663;  // vcc * 2 and resistor tolerance 
   val /= 1023;
   val -= 2731;

   if( val > 600 ) drift_defeat = 1;   // looks like osc temp measuring not working so no drift_compensation
   else drift_defeat = 0;

   if( drift_defeat ) OLED.invertText(1);
   OLED.print("osc",6*14,3*8);
   OLED.invertText(0);
   OLED.printNumI( val/10,6*18,3*8,2,' ');
   OLED.putch(94+32);  // degree font
}

void pa_temp_display( int val ){

  
   val *= 6663;  // vcc * 2 and resistor tolerance 
   val /= 1023;
   val -= 2731;

   OLED.print("pa ",6*14,3*8);
   OLED.printNumI( val/10,6*18,3*8,2,' ');
   OLED.putch(94+32);  // degree font
}




// load the table produced by clock builder into the clock chip
void si5351_init(){
 
  int i;
  
  i2cd( SI5351, 3, 0xff );  // disable outputs
  for( i = 16; i < 24; ++i ) i2cd( SI5351, i, 0x80 );  // power down output drivers
  i2flush();
  
  for( i= 15; i <= 92; i++ ){                        // load table values 
    i2cd( SI5351, i, si_clk_initial_values[ 2*i + 1 ]);
    i2flush(); 
  }
  for( i= 149; i <= 170; i++ ){
    i2cd( SI5351, i, si_clk_initial_values[ 2*i + 1 ]);
    i2flush();
  }
  i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset
  i2cd( SI5351, 3, 0xff ^ 0x5 );     // enable CLK0 and CLK2 outputs
  i2flush();
}


//    I2C handler for blind writes  
//    multiple writes indirect via pointer to char array
//void i2ci( unsigned char addr, unsigned char reg, unsigned char *p, int count ){

//    Wire.beginTransmission( addr );
//    Wire.send( reg );                      // starting register or 1st data if no registers in device
//    while( count-- )  Wire.send( *p++ );
//    Wire.endTransmission();
//}
//    single write direct
void i2cd( unsigned char addr, unsigned char reg, unsigned char dat ){

    i2start( addr );
    i2send( (int) reg );                    // register or 1st data byte if no registers in device
    i2send( (int) dat );
    i2stop();
}

// general idea of this code from some code written by Hans Summers
// modified to remove redundant division, and merge two functions into one
// load a new frequency into PLL A or B 
// the output divider is fixed per band in use and not recalculated
void  si_pll_x(unsigned char pll, uint64_t freq, int out_divider ){
 uint64_t a,b,c;
 uint64_t bc128;             // floor 128 * b/c term of equations
 uint64_t pll_freq;

 uint32_t P1;            // PLL config register P1
 uint32_t P2;            // PLL config register P2
 uint32_t P3;            // PLL config register P3
 uint64_t r;
   
   c = 1000000;     // max 1048575
   pll_freq = freq * out_divider;
   a = pll_freq / (clock_freq - 100*temp_correction);
   r = pll_freq - a * (clock_freq - 100*temp_correction);
   b = ( c * r ) / (clock_freq - 100*temp_correction);
   bc128 =  (128 * r)/ (clock_freq - 100*temp_correction);
   P1 = 128 * a + bc128 - 512;
   P2 = 128 * b - c * bc128;
   if( P2 > c ) P2 = 0;        // ? avoid negative numbers 
   P3 = c;

// get this working and then change to use i2ci table method, less writes
   i2cd(SI5351, pll + 0, (P3 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 1, (P3 & 0x000000FF));
   i2cd(SI5351, pll + 2, (P1 & 0x00030000) >> 16);
   i2cd(SI5351, pll + 3, (P1 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 4, (P1 & 0x000000FF));
   i2cd(SI5351, pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
   i2cd(SI5351, pll + 6, (P2 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 7, (P2 & 0x000000FF));
   i2flush();
   
 //  i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset  
}


// was simple code to only load output 0 integer divider
// just made it more complicated for rx - tx switching
// init has loaded other registers with the clock builder values to allow this simplified code to work
void si_load_divider( int val, int clk , int rst){
 
   val = 128 * val - 512;
   i2cd( SI5351, 44+8*clk, (val >> 16 ) & 3 );
   i2cd( SI5351, 45+8*clk, ( val >> 8 ) & 0xff );
   i2cd( SI5351, 46+8*clk, val & 0xff );   
   if( rst ) i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset needed?
}

// read the encoder and return -1,0,1
int encoder(){
   static int dir;
   static int code;
   const int codes[] = {0,1,3,2,0,1};
   // const int codes[] = {0,2,3,1,0,2};     // reversed wired the encoder
   int newcode;
   int i,j;
   
   newcode =  PORTE & 3;
   if( code == newcode ) return 0;   // no movement
   
   // get index in table of code and newcode - skip zero index for code
   for( i = 1; i < 5; ++i ){
     if( code == codes[i] ) break;
   }
   for( j= i-1; j <= i+1; ++j ){     // only look at valid code changes
     if( newcode == codes[j] ) break;
   }
   
   j= j - i;       // new direction
   if( j == 2 ) return 0;    // invalid code transition
   
   // j should be -1 or 1 at this point
   code = newcode;
   if( dir != j ){
     dir = j;
     return 0;              // debounce - require two changes in same direction 
   }
 
  return dir;
}


// set or reset tx_inhibit.  9 ranges to check for advanced class
void check_band_limits(){
int i;
int ok, cur;
unsigned long test;

    ok = 0;
    cur = tx_inhibit;
    test = (calc_vfo(TX) - bfo)/100;
    
    for( i = 0; i < 9; ++i ){
       if( test > band_lim[i].start_ && test < band_lim[i].end_ ) ok = 1;
    }
    tx_inhibit = ( ok == 0 )? 1 : 0;
    if( cur != tx_inhibit ) status_display();    
}

void set_power(){
int pwr;
  
  pwr = ( tx_inhibit ) ?  0 : band_info[band].power;    // will this be milliwatts out of band?
  pwr = map(pwr,0,100,4095,2000);     // was 4095,2800 before -8 volt agc adjustment
  pwr &= 0xfff;
  i2cd( D2A_POWER, pwr >> 8, pwr & 0xff ); 
}

//  Load the qrp-labs PA R-2R D/A on virtual pins 3,4,5
//  passed val is desired voltage * 10 and that is mapped to the dac value
void tx_load( int val ){  
  
int mask;  
  // looks like the data is msb 1st
  // writes staggered for conversion to set/reset registers
  // 25 meg clocks, 25ns or so setup and hold times
  val = map(val,0,236,0,255);   // convert volts to dac. Max volts is 23.6.

  mask = 0x80;
  while(mask){
    LATDCLR = 1;       //digitalWrite(SCLK,LOW);
    if(val & mask )  LATDSET = 2; // digitalWrite(SER,HIGH);
    else LATDCLR = 2; //digitalWrite(SER,LOW);
    mask >>= 1;
    LATDSET = 1;       //digitalWrite(SCLK,HIGH);
  }
    LATFSET = 2;  //digitalWrite(RCLK,HIGH);
    LATDCLR = 1;  //  digitalWrite(SCLK,LOW);
    LATFCLR = 2;  //digitalWrite(RCLK,LOW);
}


void set_ifgain(){
int val;

  val = map(ifgain,0,100,4095,0);  
  i2cd( D2A_POWER, (val >> 8) & 0xf, val & 0xff );   
}

void tx_on( ){
byte temp;
uint64_t bfo_tx;

  // set power level and move bfo if in CW mode
  set_power();
  temp = 0;
  bfo_tx = 900000000;
  
  if( mode == CW ){
    bfo_tx -= cw_offset;
    temp |= (CWU + DIG_SEL);        // unbalance 1st mixer, select sidetone in audio mux
  }
  if( mode == WSPR ){
    bfo_tx -= wspr_offset;
    temp |= CWU;
  }
  
  vfo = calc_vfo(TX);
  transmitting = 1;
  tx_holdoff = 20;   // 20ms key debounce
  
  // swap bfo and vfo.   swap pll and dividers
//  i2cd(SI5351, 16, 0x4f);
//  i2cd(SI5351, 18, 0x6f);
  si_pll_x( PLLA, vfo, band_info[band].divider);
  si_pll_x( PLLB, bfo_tx, bfo_divider );
  si_load_divider( band_info[band].divider,2,0 );
  si_load_divider( bfo_divider,0,1 );
  
  temp |= TR_RELAY;
  i2cd( IO_EXP1, OLAT, temp );  
  temp |= AB_DRIVER;
  i2cd( IO_EXP1, OLAT, temp ); 
  expander1 |= temp;           // expander transmit value saved in lower byte 
  
  tx_indx = 48;   tx_cnt = 32;  
}

void tx_off(int from_toggle ){
  
  // tx_load(0);
  if( from_toggle ){    // need to power down the PA. 
    tx_indx = 16;       // if from the paddles the PA is already powered down and
    tx_cnt = 32;        // don't want to power it up again with the cosine shaping
  }
  
  if( mode == CW ){
    // disable a sidetone flag
  }
  vfo = calc_vfo(RX); 
  
  transmitting = 0;  
  tx_holdoff = 20;
  
  // swap bfo and vfo back to normal
//  i2cd(SI5351, 18, 0x4f);
//  i2cd(SI5351, 16, 0x6f);
  si_pll_x( PLLB, vfo, band_info[band].divider);
  si_pll_x( PLLA, bfo, bfo_divider );
  si_load_divider( band_info[band].divider,0,0 );
  si_load_divider( bfo_divider,2,1 );

  set_ifgain();       // the same D/A converter controls tx power and if gain
  // restore the io_exp1 value
  i2cd( IO_EXP1, OLAT, expander1 >> 8 );
  
//OLED.print(" R ",CENTER,40);  
}


void cw_on(){
  
  if( transmitting == 0 ) tx_on();
  else{
    expander1 |= AB_DRIVER;
    i2cd( IO_EXP1, OLAT, expander1 & 0xff ); 
    tx_indx = 48;  tx_cnt = 32; 
  }
  // enable a sidetone flag for the core timer routine to use to gen a sidetone
  // also the timer routine will need to do the raised cosine output modulation
  // will maybe want to move the CWU enable down here if have a backwave
  
  semi_breakin_timer = 600;   // refresh the semi breakin timer value so it doesn't time out.
                              // may want this value in a menu so is adjustable rather than magic number
}


void cw_off(){
  
  expander1 ^= AB_DRIVER;
  i2cd( IO_EXP1, OLAT, expander1 & 0xff );  //!!! this should really be done at end of the raised cosine
                                            //!!! unless the AB driver will hold the tx on long enough
                                            //!!! due to delay in I2C command processing
                                            //!!! check with the o-scope
  tx_indx = 16;   tx_cnt = 32;              // flags to gen the raised cosine tx off waveform
  
  // clear the sidetone flag
}


//  !!!!! DIT DAH need the real mask values for whatever port the paddle is wired to or readpaddle converts them to
#define DIT 1
#define DAH 2
int readpaddle(){


  return 0;
}

// called once per millisecond
void paddles(){       /* should be close to CMOS mode B */
static int state = 0;
static int halftime;  // now 1/3 time, a little more delay before capturing the iambic last element or mode B
static int mask;
static int time;
static int counter;
static int cel,nel;   // current element, next element

   switch (state){
     case 0:          /* handle next element or get a new one */
       cel = nel,  nel = 0;  
       if( cel == 0 ) cel = readpaddle();
       if( cel == (DIT + DAH) ) cel = DIT;  /* dit wins on a tie */
       if( cel ){     /* key up */
          cw_on();
          mask= ( DIT + DAH ) ^ cel;
          time = 1200/wpm;
          if( cel == DAH ) time *= 3;
          halftime= time/3 ;
          state = 1;
          counter = 0;
       } 
     break;
     
     case 1:        /* timing half character */
        if( counter >= halftime ) state = 2;
     break;
        
     case 2:       /* sample opposite paddle */
        if( nel == 0 ) nel= readpaddle() & mask;
        if( counter >= time ){
           cw_off();
           state = 3;
           counter = 0;
           time = 1200/wpm;
        }
     break;
        
     case 3:    /* timing inter element gap */  // !!! should mask bit dit+dah now?
        if( nel == 0 ) nel= readpaddle() & mask;
        if( counter >= time ) state= 0;
     break;        
   }  /* end switch */
}


/*******************   stage buffer to avoid serial.print blocking ****************/

int un_stage(){    /* send a char on serial */
char c;

   if( stg_in == stg_out ) return 0;
   c = stg_buf[stg_out++];
   stg_out &= ( STQUESIZE - 1);
   Serial.write(c);
   return 1;
}


void stage( unsigned char c ){
  
  stg_buf[stg_in++] = c;
  stg_in &= ( STQUESIZE - 1 );
}

void stage_str( String st ){
int i;
char c;

  for( i = 0; i < st.length(); ++i ){
     c= st.charAt( i );
     stage(c);
  }    
}

void stage_num( int val ){   /* send number in ascii */
char buf[35];
char c;
int i;

   itoa( val, buf, 10 );
   i= 0;
   while( c = buf[i++] ) stage(c);  
}

void stage_long_str( long val ){
char buf[35];
char c;
int i;

   ltoa( val, buf, 10 );
   i= 0;
   while( c = buf[i++] ) stage(c);  
}

/************************************************************************/

      //  K3 emulation code
// !!!!!!comment out errors for now - clean up when working            
void get_freq(unsigned long vfo ){   /* report vfo */

    //if( mode == CW && fun_selected[0] != WIDE ) vfo = vfo + (( sideband == USB ) ? mode_offset : - mode_offset);     
    stage_str("000");
    if( vfo < 10000000 ) stage('0');
    stage_num(vfo);  
}

void radio_control() {

static String command = "";
String lcommand;
char c;
int ritl;    // local copy of rit
int sm;
// int bat;  /* put battery voltage in front panel revision */

    if (Serial.available() == 0) return;
    
    while( Serial.available() ){
       c = Serial.read();
       command += c;
       if( c == ';' ) break;
    }
    
    if( c != ';' ) return;  /* command not complete yet */
  
    lcommand = command.substring(0,2);
 
    if( command.substring(2,3) == ";" || command.substring(2,4) == "$;" || command.substring(0,2) == "RV" ){      /* it is a get command */
      stage_str(lcommand);  /* echo the command */
      if( command.substring(2,3) == "$") stage('$');
      
      if (lcommand == "IF") {
/*
RSP format: IF[f]*****+yyyyrx*00tmvspbd1*; where the fields are defined as follows:
[f] Operating frequency, excluding any RIT/XIT offset (11 digits; see FA command format)
* represents a space (BLANK, or ASCII 0x20)
+ either "+" or "-" (sign of RIT/XIT offset)
yyyy RIT/XIT offset in Hz (range is -9999 to +9999 Hz when computer-controlled)
r 1 if RIT is on, 0 if off
x 1 if XIT is on, 0 if off
t 1 if the K3 is in transmit mode, 0 if receive
m operating mode (see MD command)
v receive-mode VFO selection, 0 for VFO A, 1 for VFO B
s 1 if scan is in progress, 0 otherwise
p 1 if the transceiver is in split mode, 0 otherwise
b Basic RSP format: always 0; K2 Extended RSP format (K22): 1 if present IF response
is due to a band change; 0 otherwise
d Basic RSP format: always 0; K3 Extended RSP format (K31): DATA sub-mode,
if applicable (0=DATA A, 1=AFSK A, 2= FSK D, 3=PSK D)
*/      
        get_freq(freq + xit);
        stage_str("     ");
        ritl= rit;
        if( ritl >= 0 ) stage_str("+0");
        else{
          stage_str("-0"); 
          ritl = - ritl;
        }
        if( ritl < 100 ) stage('0');
        if( ritl < 10 ) stage('0');    //IF[f]*****+yyyyrx*00tmvspbd1*;
        stage_num(ritl);
        stage_str("10 0003");    /* rit,xit,xmit,cw mode */
      //  if( split == 2 ) stage_str("10");
       /* else */ stage_str("00");
      //  if( split ) stage('1');
       /* else */ stage('0');
        stage_str("001 ");      
      }
      else if(lcommand == "FA") get_freq( freq );
      else if(lcommand == "FB") get_freq( freq + xit );
      else if(lcommand == "RT") { if(rit_state) stage('1'); else stage('0'); }
      else if(lcommand == "FR") stage('0');
      else if(lcommand == "FT"){
        // if( split ) stage('1');
        /* else */ stage('0');
      }
      else if( lcommand == "KS"){
        stage('0');
        stage_num(wpm);
      }
      else if(lcommand == "XF") stage_num(2);     // bandwidth fun_selected[0]);
      else if(lcommand == "AG") stage_str("030");
      else if(lcommand == "RG") stage_str("250");
      else if(lcommand == "PC") stage_str("005");
      else if(lcommand == "FW") {stage_str("0000") ; stage_num(2);}    //stage_num(fun_selected[0]);
      else if(lcommand == "IS") stage_str("0000");
      else if(lcommand == "AN") stage('1');
      else if(lcommand == "GT") stage_str("004");
      else if(lcommand == "TQ") stage_num(0);   //stage_num(transmitting);
      else if(lcommand == "PA" || lcommand == "XT" || lcommand == "NB" ) stage('0');
      else if(lcommand == "RA") stage_str("00");
      else if(lcommand == "OM") stage_str("-----F------");
      else if(lcommand == "LK") stage_num( 0 );   //stage_num(user[LOCK]);
      else if(lcommand == "MD") stage_num( 3-mode );   // stage('3');
      else if(lcommand == "RV" && command.substring(2,3) == "F"){  /* battery voltage in the revision field */
        stage(command.charAt(2));
       // bat = battery(0);
       // stage_num(bat/10);
       // stage('.');
       // stage_num(bat % 10);
        stage('0');
      }
      else if(lcommand == "RV" && command.substring(2,3) == "A"){  /* swap status in revision field */
        stage(command.charAt(2));
       // if( split == 2 ) stage_str("SWAP ");
        /*else*/ stage_str("    ");
      }
      else if(lcommand == "RV"){   // revisions
        stage(command.charAt(2));
        stage_str("     ");
      }
      else if(lcommand == "SM"){
        stage_str("00");
        sm =  9;   //smeter(0);
        if( sm < 10 ) stage('0');
        stage_num(sm);
      }   
      else{
         stage('0');  /* don't know what it is */
      }
 
    stage(';');   /* response terminator */
    }
    
    else  set_k3(lcommand,command);    /* else it is a set command ? */
   
    command = "";   /* clear for next command */
}


void set_k3(String lcom, String com ){
String arg;
long val;
char buf[25];

 
    if( lcom == "FA" || lcom == "FB" ){    /* set vfo freq */
      arg = com.substring(2,13);
      arg.toCharArray(buf,25);
      val = atol(buf);
  //    if( mode == CW && fun_selected[0] != WIDE ) val = val - (( sideband == USB ) ? mode_offset : - mode_offset);     
        cat_band_change((unsigned long)val);
        if( lcom == "FB" && xit_state ) xit = freq - val;
        if( lcom == "FA" ) freq = val;
        freq_display(freq);
    }
    else if( lcom == "KS" ){    /* keyer speed */
      arg= com.substring(2,5);
      arg.toCharArray(buf,25);
      val = atol(buf);
      wpm = val;
    }
    else if( lcom == "LK" ){     /* lock vfo's */
    //  val = com.charAt(2);
    //  if( val == '$' ) val = com.charAt(3);
    //  user[LOCK] = val - '0';
    }
    else if( lcom == "FW" ){     /* xtal filter select */
      val = com.charAt(6) - '0';
      if( val < 4 && val != 0 ){
     //   fun_selected[0] = val;
     //   set_band_width(val);
      }
    }
    else if( lcom == "FT" ){     /* enter split */
      val = com.charAt(2) - '0';
      if( val == 0 ){
     //   if( split == 2 ) rx_vfo = tx_vfo;
     //   else tx_vfo = rx_vfo;        
      }
    //  split = user[SPLIT] = val;
    //  user[SWAPVFO] = 0;        
    }
    else if( lcom == "FR" ){    /* cancel split ? */
      val = com.charAt(2);
      if( val == '0' ){
      //  if( split == 2 ) rx_vfo = tx_vfo;
       // else tx_vfo = rx_vfo;
       // split = user[SPLIT] = user[SWAPVFO] = 0;
      }
    }
    else if( com == "SWT11;" ){    /* A/B tap. swap (listen) vfo */
    //  if( split < 2 ){            /* turns on split if off */
    //    split = 2;
    //    user[SPLIT]= user[SWAPVFO] = 1;
    //  }
     // else{                        /* back to listen on RX freq, stay in split */
     //  split = 1;
     //  user[SWAPVFO] = 0;
     // }
     // update_frequency(DISPLAY_UPDATE);  /* listen on selected vfo */
    } 
                
}


void tune_band_change( ){    // if tune past band region, switch to another filter and band registers
int b;                       // not saving the band info, so if band switch will go back to last save and not the band edge
    
    b = check_band(freq);
    if( b != band ){    // band change
       band = b;
       band_change2();  
    }
}


void cat_band_change( uint64_t val ){    /* detect if we have a large qsy */
int b;
    
    b = check_band(val);
    if( b != band ){    // band change
       band = b;
       freq = val;
       rit = band_info[band].rit;
       xit = band_info[band].xit;
       rit_state = band_info[band].rit_state;
       xit_state = band_info[band].xit_state;
     //  stp = band_info[band].stp;
       mode = band_info[band].mode;
      // tone_offset = band_info[band].tone_offset;
       band_change2();  
    }
}

int check_band( uint64_t val ){
int b;
 
    b= 0;
    val /= 100;
    if( val < 5000000 ) b = 0; 
    if( val >= 5000000 && val < 6000000) b= 1; 
    if( val >= 6000000 && val < 8000000) b= 2;
    if( val >= 8000000 && val < 12000000 ) b= 3;
    if( val >= 12000000 && val < 16000000 ) b= 4;
    if( val >= 16000000  ) b= 5;
    return b;
}


        /* end of K3 emulation functions */

/***********************************************************************/


/*  Core function - calculate the value to return as period in us * 40 */
/*    or 1/freq  *  40000000 */

//   lowpass fir decimation filter constants, 3000 cutoff 4000 stop at 16k rate
long fc[ 32 ] = { 
// 0 . 006820q15 , 
 223,
// , 0 . 011046q15 , 
 361,
// , - 0 . 004225q15 , 
 -138,
// , - 0 . 016811q15 , 
  -550,
// , - 0 . 001921q15 , 
  -62,
// , 0 . 021797q15 , 
  714,
// , 0 . 012515q15 , 
  410,
// , - 0 . 024048q15 , 
  -788,
// , - 0 . 028414q15 , 
  -931,
// , 0 . 020730q15 , 
  679,
// , 0 . 051289q15 , 
  1680,
// , - 0 . 006587q15 , 
  -215,
// , - 0 . 087749q15 , 
  -2875,
// , - 0 . 036382q15 , 
  -1192,
// , 0 . 186091q15 , 
  6097,
// , 0 . 403613q15 , 
  13225,
// , 0 . 403613q15 , 
  13225,
// , 0 . 186091q15 , 
  6097,
// , - 0 . 036382q15 , 
  -1192,
// , - 0 . 087749q15 , 
  -2875,
// , - 0 . 006587q15 , 
  -215,
// , 0 . 051289q15 , 
  1680,
// , 0 . 020730q15 , 
  679,
// , - 0 . 028414q15 , 
  -931,
// , - 0 . 024048q15 , 
  -788,
// , 0 . 012515q15 , 
  410,
// , 0 . 021797q15 , 
  714,
// , - 0 . 001921q15 , 
  -62,
// , - 0 . 016811q15 , 
  -550,
// , - 0 . 004225q15 , 
  -138,
// , 0 . 011046q15 , 
  361,
// , 0 . 006820q15  
  223
};


// cw filter 400 hz wide centered at about 600 hz
int fcw[ 32 ] = { 
// - 0 . 000426q15 , 
	  -13,
// , - 0 . 003116q15 , 
	  -102,
// , - 0 . 005389q15 , 
	  -176,
// , - 0 . 003948q15 , 
	  -129,
// , 0 . 001048q15 , 
	  34,
// , 0 . 004523q15 , 
	  148,
// , - 0 . 001170q15 , 
	  -38,
// , - 0 . 020961q15 , 
	  -686,
// , - 0 . 051978q15 , 
	  -1703,
// , - 0 . 082194q15 , 
	  -2693,
// , - 0 . 094538q15 , 
	  -3097,
// , - 0 . 075093q15 , 
	  -2460,
// , - 0 . 021324q15 , 
	  -698,
// , 0 . 054266q15 , 
	  1778,
// , 0 . 127619q15 , 
	  4181,
// , 0 . 172683q15 , 
	  5658,
// , 0 . 172683q15 , 
	  5658,
// , 0 . 127619q15 , 
	  4181,
// , 0 . 054266q15 , 
	  1778,
// , - 0 . 021324q15 , 
	  -698,
// , - 0 . 075093q15 , 
	  -2460,
// , - 0 . 094538q15 , 
	  -3097,
// , - 0 . 082194q15 , 
	  -2693,
// , - 0 . 051978q15 , 
	  -1703,
// , - 0 . 020961q15 , 
	  -686,
// , - 0 . 001170q15 , 
	  -38,
// , 0 . 004523q15 , 
	  148,
// , 0 . 001048q15 , 
	  34,
// , - 0 . 003948q15 , 
	  -129,
// , - 0 . 005389q15 , 
	  -176,
// , - 0 . 003116q15 , 
	  -102,
// , - 0 . 000426q15  
	  -13,
};

uint32_t dsp_core( uint32_t timer ){
static int ad_timer;   // sample other a/d pins at different rates
static int ot;         // oversample the osc temp sensor for more bits
static int otc;        // oversample counter


   ++ad_timer;
   ad_timer &= 127;
   if( transmitting ) raised_cosine();
   else run_filters();
   
   // sample other a/d pins at a slower rate
   if( ad_timer == 1 ){
     ot += analogRead( A0 );   // over sample for 3 extra bits
     ++otc;   otc &= 63;
     if( otc == 0 ){        // new value rate a little more than 1 per second
        osc_temp = ot >> 3;
        ot = 0;
     }
   }
   if( ad_timer == 4 ) pa_temp = analogRead( A1 );
   if( (ad_timer & 0xf) == 0xf ){
     multi_pot = 7 * multi_pot + analogRead( A2 );
     multi_pot >>= 3;
   }

   LATDINV = (1 << 8);   // toggle RD8 for negative voltage supply charge pump
                         // last so noise doesn't affect the A/D readings
                         // this is virtual pin 2
   return timer + 2500;   // 2500 == 16k, 5000 == 8k sample rate, 6250 == 6400 hz sample rate
}


// called from dsp_core
void run_filters(){
long sample;
static long w[32];     // delay line
static int win;
static int flip;
int i,j;
// cw filter variables
static long wc[32];
static int wcin;


audio = sample;
   // sample = analogRead( A0 );  !!!! fix up // 10 bits in

   // run a decimation filter for 16k to 8k rate
   i = win;
   w[win++] = sample - 512;   // store in delay line, remove dc component of the signal 
   win &= 31;
   
   flip ^= 1;
   if( flip ){   // only need to run the filter at the slower rate
      sample = 0;   // accumulator 
 
      for( j = 0; j < 32; j++ ){
        sample +=  fc[j] * w[i++];
        i &= 31;
      }
   //   audio = sample >> 15;
      
      if( mode == CW ){    // run the cw filter at 8k rate
         sample = 0;
         i = wcin;
         wc[wcin++] = audio;
         wcin &= 31;
         
         for( j = 0; j < 32; j++ ){
           sample +=  fcw[j] * wc[i++];
           i &= 31;
         }     
     //    audio = sample >> 15;
      }
      
      audio_flag = 1;
   }
  
}

// called from dsp_core.   CW wave shaping.
void raised_cosine(){
static int flip;
unsigned int val;

   if( tx_cnt == 0 ) return;    // nothing to do
   flip ^= 1;
   if( flip == 0 ) return;      // running at 8k rate.  Should be 4ms to ramp up or down
   
   val = 160 * (sin_cos[tx_indx] + 4096);      // 16 volts * raised cosine value
   val >>= 13;                                 // adjust by q12 factor and double caused by raising cosine value
   tx_load(val);
   --tx_cnt;  ++tx_indx;
}



//      Download WSPRcode.exe from  http://physics.princeton.edu/pulsar/K1JT/WSPRcode.exe   and run it in a dos window 
//      Type (for example):   WSPRcode "K1ABC FN33 37"    37 is 5 watts, 30 is 1 watt, 33 is 2 watts, 27 is 1/2 watt
//      ( Use capital letters in your call and locator when typing in the message string.  No extra spaces )
//      Using the editing features of the dos window, mark and copy the last group of numbers
//      Paste into notepad and replace all 3 with "3,"  all 2 with "2," all 1 with "1," all 0 with "0,"
//      Remove the comma on the end
//  the current message is   "K1URC FN54 23"
const char wspr_msg[] = { 
 3, 3, 2, 2, 2, 0, 0, 2, 1, 2, 0, 2, 1, 1, 1, 2, 2, 2, 3, 0, 0, 1, 0, 1, 1, 3, 3, 2, 2, 0,
 2, 2, 0, 0, 3, 2, 0, 3, 0, 1, 2, 0, 2, 0, 0, 2, 3, 0, 1, 3, 2, 0, 1, 1, 2, 1, 0, 2, 0, 3,
 3, 2, 3, 2, 2, 2, 2, 1, 3, 2, 3, 0, 3, 0, 3, 0, 3, 2, 0, 3, 2, 0, 3, 0, 3, 1, 0, 2, 0, 1,
 1, 2, 1, 2, 3, 0, 2, 2, 3, 2, 2, 0, 2, 2, 1, 0, 2, 1, 2, 0, 3, 3, 1, 2, 3, 1, 2, 2, 3, 1,
 2, 1, 2, 0, 0, 1, 1, 3, 2, 2, 2, 2, 0, 1, 0, 1, 2, 0, 3, 1, 0, 2, 0, 0, 2, 2, 0, 1, 3, 0,
 1, 2, 3, 1, 0, 2, 2, 1, 3, 0, 2, 2
 };


/*   WSPR  core timer function */
// #define WSPRTICK 27307482     // 1 bit time for 1.4648 baud.  
#define WSPRTICK 27306667        // or should it be 1.46484375 baud.  120000/8192

uint32_t  wspr_core( uint32_t timer ){
static int count;
long dds_freq_base;
uint64_t bfo_tx;
const int precalc[4] = {0,146,293,439};
int i;

  if( wspr_tx_enable == 0 )   return timer + WSPRTICK;   // nothing to do

  if( count == 162 ) {   // stop the transmitter
     tx_off(1);
     wspr_tx_enable = 0;
     count = 0;
   //  update_frequency(NO_DISPLAY_UPDATE);
     return timer + WSPRTICK;
  }

  if( count == 0 ) tx_on();
    
  //dds_freq_base = ( tx_vfo + mode_offset ) * (268.435456e6 / Reference );    
  //wspr_to_freq( dds_freq_base + 8L * (long)wspr_msg[count] );
  bfo_tx = 900000000 - wspr_offset;
//  bfo_tx -= ((uint64_t)( (float)wspr_msg[count] * 146.484375 )); use the precalc values with rounding
  i = wspr_msg[count];
  bfo_tx -= precalc[i];
  si_pll_x( PLLB, bfo_tx, bfo_divider );

  ++count;
   
  return timer + WSPRTICK;    
}

unsigned long msec,oldtime;
int sec;

void reset_wspr_times( ){
 
   oldtime = millis();
   msec = sec = 0; 
}

// que up auto send wspr messages
// change to wspr mode on 2 minute mark to sync up the timer
void wspr_timer(unsigned long ms){
static int last_time;
static int wspr_que;

   msec+=  ( ms - oldtime );
   oldtime = ms;
   if( msec >= 1000 ){      // one second period
      msec -= 1000;   
      if( ++sec >= 120){    // start of a two minute period
        sec -= 120;
       // if( led_on_timer == 0 ) set_display(4,4);
        if( random(100) <  (long)( 5 * wspr_duty )) ++wspr_que;    //1 == 5%,  2 == 10% duty
        if( wspr_que ){
          if( last_time ) last_time = 0;
          else{
             wspr_tx_enable = last_time = 1;
             --wspr_que;
          }                 
        }
      }    // end of two minutes

      OLED.invertText(wspr_tx_enable);
      OLED.printNumI( sec, 6*18,1*8,3,' ');
      OLED.invertText(false);
   }  // end of a new one second

}



/*  I2C write only implementation using polling of the hardware registers */
/*  functions do not block ( the wire library start and stop functions block ) */
/*  call i2poll() in loop() to keep everything going */

// use some upper bits in the buffer for control
#define ISTART 0x100
#define ISTOP  0x200

void i2start( unsigned char adr ){
int dat;
  // shift the address over and add the start flag
  dat = ( adr << 1 ) | ISTART;
  i2send( dat );
}

void i2send( int data ){   // just save stuff in the buffer
  i2buf[i2in++] = data;
  i2in &= (I2BUFSIZE - 1);
}

void i2stop( ){
   i2send( ISTOP );   // que a stop condition
}


void i2flush(){  // needed during setup, call poll to empty out the buffer.  This one does block.

  while( i2poll() ); 
}

int i2poll(){    // everything happens here.  Call this from loop.
static int state = 0;
static int data;
static int delay_counter;

   if( delay_counter ){   // the library code has a delay after loading the transmit buffer and before the status bits are tested for transmit active
     --delay_counter;
     return (16 + delay_counter);
   }
   
   switch( state ){    
      case 0:      // idle state or between characters
        if( i2in != i2out ){   // get next character
           data = i2buf[i2out++];
           i2out &= (I2BUFSIZE - 1 );
           
           if( data & ISTART ){   // start
              data &= 0xff;
              // set start condition
              I2C1CONSET = 1;
              state = 1; 
           }
           else if( data & ISTOP ){  // stop
              // set stop condition
              I2C1CONSET = 4;
              state = 3;
           }
           else{   // just data to send
              I2C1TRN = data;
              delay_counter = 10;   // delay for transmit active to come true
              state = 2;
           }
        }
      break; 
      case 1:  // wait for start to clear, send saved data which has the address
         if( (I2C1CON & 1) == 0  ){
            state = 2;
            I2C1TRN = data;
            delay_counter = 10;
         }
      break;
      case 2:  // wait for ack/nack done and tbuffer empty, blind to success or fail
         if( (I2C1STAT & 0x4001 ) == 0){  
            state = 0;
         }
      break;
      case 3:  // wait for stop to clear
         if( (I2C1CON & 4 ) == 0 ){
            state = 0;
            delay_counter = 10;  // a little delay just for fun at the end of a sequence
         }
      break;    
   }
   
   // detect any error bits and see if can recover, advance to next start in the buffer
   // totally blind for now until we check some error bits

   if( i2in != i2out ) return (state + 8);
   else return state;
}


/*****************   MENU's *******************************/
struct MENU {
  const int no_sel;                 // number of selections
  const char title[15];             // top line - 14 char max in title
  const char choice[8][9];          // selections to display - max 8 selections - max text length is 8
};

struct MENU mode_menu = {
   6,
   "Select Mode",
   { "CW", "LSB", "USB", "AM", "Mem Tune", "WSPR" }          
};


struct MENU band_menu = {
   6,
   "Amateur Band",
   {"80", "60", "40", "30", "20", "15" }
};

struct MENU main_menu = {
  4,
  "Top Menu",
  { "Band", "Mode", "Multi 1", "Multi 2" }
};

struct MENU multi_1_menu = {
  5,
  "M 1 Function",
  { "None", "RIT", "Tx Drv", "Tone", "IF Gain" }
};

struct MENU multi_2_menu = {
  5,
  "M 2 Function",
  { "None", "RIT", "Tx Drv", "Tone", "IF Gain" }
};

// pass value 0 - init, 1 or -1 encoder, 2 = selection made
// return 0 when done
// this function is called repeatedly during the selection process
int top_menu( int function ){
static int top_sel;    // where we are in the top most menu
static int state;      // where we are in the selection process
static struct  MENU *active_menu;  // menu pointer so can have common code for all menu's
static int def_val;    // current default value in selection process

int ret_val;

   ret_val = 1;   // assume we are still busy

   if( function == 0 ) {
     top_menu2( top_sel, &main_menu, 0 );   // init a new main menu
     active_menu = &main_menu;
     def_val = top_sel;
     state = 0;
   }
   else if( function == 1 || function == -1 ){  // encoder 
      def_val = top_menu2( def_val, active_menu, function );   // move through values with the encoder
   }
   else{   // the switch was toggled, pick a selection and act upon it
       switch (state) {
         case 0:   // top menu was shown,  pick a submenu to display
            top_sel = def_val;
            switch ( top_sel ){
              case 0: active_menu = &band_menu;  def_val = band; state = 1; break;
              case 1: active_menu = &mode_menu;  def_val = mode; state = 2; break;
              case 2: active_menu = &multi_1_menu; def_val = multi1; state = 3; break;
              case 3: active_menu = &multi_2_menu; def_val = multi2; state = 4; break;              
            }
            if( transmitting && top_sel < 2 ) tx_off(1);
            def_val = top_menu2(def_val,active_menu, 0 );   // init a new submenu
         break;
         case 1:   // a new band was selected or not
            if( band != def_val ) band_change1( def_val );
            ret_val = state = 0;
         break;
         case 2:  mode = def_val;
           // turn off the bfo on AM mode to help remove audio mux leak through
           // or leave it on for AGC to work ?
             if( mode == AM ) i2cd( SI5351, 3, 0xff ^ 1 );  // just the vfo on
             else i2cd( SI5351, 3, 0xff ^ 0x5 );     // enable both clocks
             
             if( mode == WSPR ){
               reset_wspr_times();
               attachCoreTimerService( wspr_core );
             }
             else detachCoreTimerService( wspr_core );
             
            load_expander();
            ret_val = state = 0;
         break;   // mode change
         case 3:  multi1 = def_val; rit_onoff(0); ret_val = state = 0; break; 
         case 4:  multi2 = def_val; rit_onoff(0); ret_val = state = 0; break; 
         //default:  state = 0; ret_val = 0; break;  // temp
       }  // end switch
   }
   
   return ret_val;
}

// this function was changed to allow repeated calls
// move the highlighed selection through the menu
int top_menu2(int def_val, struct MENU *m, int encode){     // return the menu value selected/highlighted

static int old_val;   // this is perhaps redundant as the top menu also keeps track of the current selection

  if( encode == 0 ){   // init a new menu
     old_val= def_val;
     OLED.clrScr(1); 
     OLED.setFont( SmallFont );
     show_menu(def_val,m);
     return def_val;
  }
 
  def_val = old_val;  

     def_val = def_val + encode;     // turning the tuning knob to highlight a selection
     if( def_val < 0 ) def_val = 0;
     if( def_val >= m->no_sel ) def_val = m->no_sel -1;
     if( def_val != old_val){
       old_val= def_val;
       show_menu(def_val,m);            // show the new highlighted value 
     }    
   return def_val; 
}


// put menu on page 1, rows 4 to 7
void show_menu( int sel, struct MENU *m ){  // display menu on OLED display
int i;
static int base;

   while( sel > base + 2 ) ++base;    // move the menu window around only when needed
   while( sel < base ) --base;        // so the selected choice is in range
   if( m->no_sel <= 3 ) base = 0;     // most of the menu's will not need a base
   
   OLED.print(m->title,0,32);
   for( i= 0; i < m->no_sel; ++i ){
      if( i < base ) continue;
      if( i > base + 2 ) break;
      if( i == sel ) OLED.invertText( 1 );
      else OLED.invertText( 0 );
      OLED.clrRow( i - base + 1 + 4 );
      OLED.print(m->choice[i], 16, 8 * (i - base + 1) + 32 );
   }
   OLED.invertText(0);
 
 // show some hint on the screen if there are more choices than what is displayed
   if( base != 0 ) OLED.print("--",RIGHT,8 + 32);     // row 1 of the 2nd page ( or row 5 )
   if( (m->no_sel - base)  > 3 ) OLED.print("++",RIGHT,7 * 8);  // row 7
}


#ifdef NOWAY

///  Is this needed?
void mem_change(int change){    // toggle memory tuning mode
static int band_save;

   if( encoder_user == MEM_TUNE ){
     encoder_user = FREQ;
     if( change ==  1 /*TAP */ ){       // tap restores from where we were, dtap is m->vfo
       band = band_save;        // any other set function like band or mode also same as m->vfo
       freq = band_info[band].freq;
       rit = band_info[band].rit;
       xit = band_info[band].xit;
       rit_state = band_info[band].rit_state;
       xit_state = band_info[band].xit_state;
       stp = band_info[band].stp;
       mode = band_info[band].mode;
       band_change2();  
     }
     status_display();
     freq_display(freq);
   }
   else{        // load up the working values from the memory
     encoder_user = MEM_TUNE;
     // need to save our current band info in order to come back to the same place when we press vfo to exit mem tune 
     band_info[band].freq = freq;
     band_info[band].rit = rit;
     band_info[band].xit = xit;
     band_info[band].rit_state = rit_state;
     band_info[band].xit_state = xit_state;
     band_info[band].stp = stp;
     band_info[band].mode = mode;
     rit = xit = rit_state = xit_state = 0;
     band_save = band;
     mem_tune_user(0);     
   }  
}
#endif



