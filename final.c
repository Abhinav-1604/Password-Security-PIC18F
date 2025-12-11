#include <xc.h>
#include <stdint.h>
#include <string.h>

#define _XTAL_FREQ 16000000UL

#pragma config OSC = HS
#pragma config WDT = OFF
#pragma config LVP = OFF
#pragma config PWRT = ON
#pragma config BOR = ON
#pragma config DEBUG = OFF

#define RS LATBbits.LATB0
#define EN LATBbits.LATB1
#define D4 LATBbits.LATB4
#define D5 LATBbits.LATB5
#define D6 LATBbits.LATB6
#define D7 LATBbits.LATB7

#define ROW1 LATCbits.LATC0
#define ROW2 LATCbits.LATC1
#define ROW3 LATCbits.LATC2
#define ROW4 LATCbits.LATC3

#define COL1 PORTCbits.RC4
#define COL2 PORTCbits.RC5
#define COL3 PORTCbits.RC6

#define LED_RED LATDbits.LATD2
#define LED_GREEN LATDbits.LATD3

#define PASS_LEN 4
static char password[PASS_LEN+1] = "4269";  


static const char keyMap[4][3] = {
    {'1','2','3'},
    {'4','5','6'},
    {'7','8','9'},
    {'*','0','#'}
};

static uint8_t key_old = 0xFF;


static void delay_ms(uint16_t ms){ while(ms--) __delay_ms(1); }

static inline uint8_t save_row_tris(void){ return TRISC & 0x0F; }
static inline void rows_tristate(void){ TRISC |= 0x0F; }                 
static inline void rows_drive_low(void){ LATC &= ~0x0F; TRISC &= ~0x0F; } 
static inline void rows_restore(uint8_t v){ LATC &= ~0x0F; TRISC = (TRISC & 0xF0) | (v & 0x0F); }

static void lcd_set_nibble(uint8_t n){
    D4 = (n>>0)&1;
    D5 = (n>>1)&1;
    D6 = (n>>2)&1;
    D7 = (n>>3)&1;
}
static void lcd_pulse_en(void){ EN=1; __delay_us(50); EN=0; delay_ms(2); }

static void lcd_send_byte_safe(uint8_t b, uint8_t is_data){
    uint8_t saved = save_row_tris();
    rows_tristate();
    RS = is_data ? 1 : 0;
    lcd_set_nibble((b>>4)&0x0F); lcd_pulse_en();
    lcd_set_nibble(b & 0x0F); lcd_pulse_en();
    RS = 0;
    rows_restore(saved);
}
static void lcd_cmd_safe(uint8_t c){ lcd_send_byte_safe(c, 0); }
static void lcd_putc_safe(char ch){ lcd_send_byte_safe((uint8_t)ch, 1); }
static void lcd_puts_safe(const char *s){ while(*s) lcd_putc_safe(*s++); }
static void lcd_clear_safe(void){ lcd_cmd_safe(0x01); delay_ms(2); }
static void lcd_goto_safe(uint8_t row, uint8_t col){
    uint8_t addr = (row==1)?0x80:0xC0;
    lcd_cmd_safe(addr + (col-1));
}
static void lcd_init(void){
    LATB = 0x00;
    LATC &= ~0x0F;
    TRISBbits.TRISB0=0; TRISBbits.TRISB1=0;
    TRISBbits.TRISB4=0; TRISBbits.TRISB5=0;
    TRISBbits.TRISB6=0; TRISBbits.TRISB7=0;
    RS=0; EN=0;
    delay_ms(20);
    lcd_set_nibble(0x03); lcd_pulse_en(); delay_ms(5);
    lcd_set_nibble(0x03); lcd_pulse_en(); __delay_us(200);
    lcd_set_nibble(0x03); lcd_pulse_en(); __delay_us(200);
    lcd_set_nibble(0x02); lcd_pulse_en(); __delay_us(200);
    lcd_cmd_safe(0x28); lcd_cmd_safe(0x0C); lcd_cmd_safe(0x06); lcd_clear_safe();
}


static void keypad_init(void){
    LATC &= ~0x0F;
    TRISC = 0x70;
    rows_drive_low();
}
static void wait_key_release(void){
    uint16_t tmo=2000;
    while((COL1||COL2||COL3) && tmo--){
        rows_drive_low();
        delay_ms(1);
    }
    delay_ms(40);
    key_old = 0xFF;
}
static char keypad_getc(void){
    uint8_t found = 0xFF;
    for(uint8_t r=0;r<4;++r){
        rows_drive_low();
        if(r==0) ROW1=1; else if(r==1) ROW2=1; else if(r==2) ROW3=1; else ROW4=1;
        __delay_us(60);
        if(COL1) found=r*3+0;
        else if(COL2) found=r*3+1;
        else if(COL3) found=r*3+2;
        if(found!=0xFF) break;
    }
    rows_drive_low();
    if(found==0xFF){ key_old=0xFF; return 0xFF; }
    delay_ms(15);
    rows_drive_low();
    uint8_t r_index = found / 3;
    ROW1 = ROW2 = ROW3 = ROW4 = 0;
    if(r_index==0) ROW1=1; else if(r_index==1) ROW2=1; else if(r_index==2) ROW3=1; else ROW4=1;
    __delay_us(60);
    uint8_t conf = 0xFF;
    if(COL1) conf = r_index*3 + 0;
    else if(COL2) conf = r_index*3 + 1;
    else if(COL3) conf = r_index*3 + 2;
    rows_drive_low();
    if(conf==0xFF) return 0xFF;
    uint16_t rel_tmo = 1500; while((COL1||COL2||COL3) && rel_tmo--) delay_ms(1);
    delay_ms(30);
    if(conf == key_old) return 0xFF;
    key_old = conf;
    return keyMap[conf/3][conf%3];
}

static void leds_init(void){
    TRISDbits.TRISD2=0; TRISDbits.TRISD3=0;
    LED_RED=0; LED_GREEN=0;
}
static void indicate_success_short(void){
    uint8_t s = save_row_tris();
    rows_tristate();
    LED_RED=0; LED_GREEN=1;
    delay_ms(2000);
    LED_GREEN=0;
    rows_restore(s);
}
static void indicate_failure_short(void){
    uint8_t s = save_row_tris();
    rows_tristate();
    for(uint8_t i=0;i<3;i++){ LED_RED=1; delay_ms(250); LED_RED=0; delay_ms(200); }
    rows_restore(s);
}

static void read_digits_masked(char *buf){
    uint8_t idx = 0;
    memset(buf, 0, PASS_LEN+1);
    lcd_goto_safe(2,1);
    while(idx < PASS_LEN){
        char k = keypad_getc();
        if(k == 0xFF){ delay_ms(5); continue; }
        if(k == '*'){
            if(idx){
                idx--;
                buf[idx] = 0;
                
                lcd_goto_safe(2,1);
                for(uint8_t t=0;t<idx;t++) lcd_putc_safe('*');
                for(uint8_t t=idx;t<PASS_LEN;t++) lcd_putc_safe(' ');
                lcd_goto_safe(2,1+idx);
            }
            continue;
        }
        if(k == '#'){
            continue;
        }
        buf[idx++] = k;
        lcd_putc_safe('*');
    }
    buf[PASS_LEN] = '\0';
}

static void change_password_flow(void){
    char buf[PASS_LEN+1];

    lcd_clear_safe();
    lcd_goto_safe(1,1);
    lcd_puts_safe("Change Pass:");
    lcd_goto_safe(2,1);
    lcd_puts_safe("Prepare...");
    delay_ms(400);

    lcd_clear_safe();
    lcd_goto_safe(1,1); lcd_puts_safe("Enter Old:");
    lcd_goto_safe(2,1);
    wait_key_release();
    read_digits_masked(buf);

    if (strncmp(buf, password, PASS_LEN) != 0){
        lcd_clear_safe();
        lcd_goto_safe(1,1); lcd_puts_safe("Wrong Old Pass");
        lcd_goto_safe(2,1); lcd_puts_safe("Returning...");
        indicate_failure_short();
        wait_key_release();
        delay_ms(800);
        return;
    }

    lcd_clear_safe();
    lcd_goto_safe(1,1); lcd_puts_safe("Enter New Pass");
    lcd_goto_safe(2,1);
    wait_key_release();
    read_digits_masked(buf);

    memcpy(password, buf, PASS_LEN);
    password[PASS_LEN] = '\0';

    lcd_clear_safe();
    lcd_goto_safe(1,1); lcd_puts_safe("Password Saved");
    lcd_goto_safe(2,1); lcd_puts_safe("Returning...");
    indicate_success_short();
    wait_key_release();
    delay_ms(500);
}
void main(void){
    ADCON1 = 0x0F;

    LATB = 0x00;
    LATC &= ~0x0F;

    lcd_init();
    keypad_init();
    leds_init();

    lcd_clear_safe();
    lcd_goto_safe(1,1);
    lcd_puts_safe("Enter Password:");
    lcd_goto_safe(2,1);
    wait_key_release();

    char input[PASS_LEN+1];
    uint8_t idx = 0;

    while (1){
        char k = keypad_getc();
        if (k == 0xFF) continue;

        if (k == '#'){ 
            wait_key_release();
            change_password_flow();
            lcd_clear_safe();
            lcd_goto_safe(1,1); lcd_puts_safe("Enter Password:");
            lcd_goto_safe(2,1);
            wait_key_release();
            idx = 0; memset(input,0,sizeof(input));
            continue;
        }

        if (k == '*'){ 
            if (idx){
                idx--;
                input[idx] = 0;
                lcd_goto_safe(2,1);
                for (uint8_t t=0;t<idx;t++) lcd_putc_safe('*');
                for (uint8_t t=idx;t<PASS_LEN;t++) lcd_putc_safe(' ');
                lcd_goto_safe(2,1+idx);
            }
            continue;
        }

        if (idx < PASS_LEN){
            input[idx++] = k;
            lcd_putc_safe('*');
        }

        if (idx >= PASS_LEN){
            input[PASS_LEN] = '\0';
            delay_ms(150);

            if (strcmp(input, password) == 0){
                lcd_clear_safe();
                lcd_goto_safe(1,1);
                lcd_puts_safe("Correct Password");
                lcd_goto_safe(2,1);
                lcd_puts_safe("Access Granted");
                indicate_success_short();
            } else {
                lcd_clear_safe();
                lcd_goto_safe(1,1);
                lcd_puts_safe("Wrong Password");
                lcd_goto_safe(2,1);
                lcd_puts_safe("Try Again...");
                indicate_failure_short();
            }

            wait_key_release();

            idx = 0;
            memset(input,0,sizeof(input));
            lcd_clear_safe();
            lcd_goto_safe(1,1);
            lcd_puts_safe("Enter Password:");
            lcd_goto_safe(2,1);
        }
    }
}
