/*
*       Andrzej Zieliński
*
*     czujnik dymu 2018 XII
*
*    ATmega328P + MQ2 + LCD
*/


#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define LCD_ddr DDRB //porty LCD - rs: PB0, en: PB1, linie danych: PB2..5
#define LCD_port PORTB
#define LCD_rs PB0
#define LCD_en PB1
#define LCD_bckl (1 << PD3)
#define SW1 (1 << PD2) //switch
#define BUZZ_pin PD0 //buzzer
#define LED_pin PD1 //dioda czerwona
#define LED_blue (1 << PD7) // dioda niebieska


//Deklaracje fcji

void LCD_char(int8_t); //znak na ekran
void LCD_on(); //on
void LCD_clr(); //clrscr :D
void LCD_kom(int);
void LCD_line(char*);
void LCD_text(char *, char *);
void ADC_init();
void TIMER1_init();
int16_t menu_alm_off(int16_t);
float mq2_calculate_ppm(float);

// zmienne globalne
volatile uint8_t gsp,gs,gm,gh; //godziny, minuty, sekundy,połowki sekund
uint16_t EEMEM alarm_off = 360;

int main (void)
{
    uint8_t alarm=0,gs1=0,gs2=0,gs3=0,t1=0,tabi=0,k=0,nap_h=0,nap_l=0,p=0,p1=0,p2=0;
    uint16_t sw_lock=0,t=0;
    uint16_t pomiar,pom[4]={},nap_tab[10]={},poz=0;
    uint32_t nap_adc=0,nap_adc_adc=0;
    //int16_t alm_off=360;
    int16_t alm_off;
    alm_off = eeprom_read_word(&alarm_off);
    //buzzer / LED
    DDRD |= (1<<BUZZ_pin)|(1<<LED_pin)|LED_blue;
    //DDRC |= (1<<PC2)|(1<<PC3); //jako wyjscia, dla poprawy pomiarow adc
    PORTD |= (1<<PC2)|(1<<PC3);
    PORTD &= ~((1<<BUZZ_pin)|(1<<LED_pin)|LED_blue);
    //
    LCD_on();
    LCD_clr();
    // podswietlenie ON
    DDRD |= LCD_bckl;
    PORTD |= LCD_bckl;
    // definiujemy przycisk
    DDRD &= ~SW1; // wejscie
    PORTD |= SW1; // pull up

    char napis[40], komunikat[40]="Nagrzewanie     ";
    LCD_line(komunikat);
    for (int8_t i=0; i<16; i++) //pętla opóźniająca (nagrzewanie czujnika)
    {
        LCD_char('.');
        _delay_ms(150);
    }

    //sprintf(komunikat,"Pomiar powietrza   ");
    TIMER1_init();
    ADC_init();
    sei(); // włączenie przerwań
    _delay_ms(200);
    while(1) //petla glowna
    {
        if (!sw_lock && !(PIND & SW1)) // sprawdzam przycisk
        {
            if (!alarm)
            {
                PORTD ^= LCD_bckl;
            }
            else // kiedy alarm wyłącz led i buzz
            {
                alarm=0;
            }
            sw_lock = 50000;
            gs3=gs;
        }
        else if (sw_lock && (PIND & SW1)) sw_lock++; // zapobiegaganie wielokrotnemu czytaniu + eleiminacja drgan (15535 przebiegów)
        else if (!(PIND & SW1) && gs3 != gs)
            {
                gs3=gs;
                if (t1++ == 1) PORTD |= LCD_bckl;
                if (t1 == 2)
                {
                    alm_off = 2*menu_alm_off(alm_off/2); // bo s na połówki
                    t1=0;
                }
            }

        if ((gsp == 0) && (gs1 == 0)) // pomiar co s  gs1-blokada powtórzeń
        {
            gs1=1; // blokada on
            ADMUX &= ~(1 << MUX0); // mux bit 0 - ADC0
            ADCSRA |= (1<<ADSC); // pojedyncza konwersja
            while(ADCSRA & (1<<ADSC)); // czekam na koniec konwersji
            //pomiar = ADC;
            pom[k] = ADC;
            if (++k >= 4) k = 0;
            pomiar = (pom[0]+pom[1]+pom[2]+pom[3])/4;

            //float volt= (pomiar*5)/1023; // ADC na V
            //float mq_res = ((5-volt)-1)*10; // V na ohm
            //float ppm = mq2_calculate_ppm(mq_res/9.8); // wywołanie fcji
            //uint16_t ppmi = (int)ppm;

           // PORTC |= (1<<PC0);
           // PORTC &= ~(1<<PC1);

            ADMUX |= (1 << MUX0); // ADC1
            ADCSRA |= (1<<ADSC); // pojedyncza konwersja
            while(ADCSRA & (1<<ADSC)); // czekam na koniec konwersji

            nap_tab[tabi] = ADC;
            //nap_adc_adc = nap_tab[tabi]; //ADC;
            if (++tabi >= 10) tabi = 0;

            if (++p>=10)
            {
                p=10;
                nap_adc=0;
                for (int i=0;i<10;i++)
                {
                    nap_adc += nap_tab[i];
                }
                nap_adc /= 10;
                nap_adc = (nap_adc*34.9*100);
                nap_adc /= 1023;
                nap_h = nap_adc/100;
                nap_l = (nap_adc%100)/10;
                //nap_adc_adc = nap_adc;
                //nap_adc = 0;
                poz=nap_h*10 + nap_l;
                if (poz < 232 && poz > 150)  // p1: 1 - dioda, 2 - dioda + brzeczyk, 0 - brak alarmu
                {
                    if (poz < 228) p1 = 2;
                    else
                        p1 = 1;
                }
                else
                    p1=0;
            }


            sprintf(komunikat,"Nap: %d.%dV            ",nap_h,nap_l);
            //sprintf(napis,"tabi: %d, adc_tab: %d     ",tabi,nap_tab[tabi]);
            sprintf(napis,"Powietrze OK %d         ",pomiar);

            if (p1 == 2)
                PORTD ^= (LED_blue | 1 << BUZZ_pin);
            else if (p1 == 1)
                PORTD ^= (LED_blue);
            else
            {
                if (!alarm)
                    PORTD &= ~(LED_blue | 1 << BUZZ_pin);
            }

            if (pomiar > 15)
            {
                if (alm_off >=0)
                    if (p2++ > 4) alarm=1; // po 4 pomiarach > 35
                t=0;
                sprintf(napis,"GAZ!!! %d        ",pomiar);
            }
            else if ((int16_t)++t==alm_off) // po określonym czasie alarm off
            {
                t = 0;
                p2 = 0;
                alarm = 0;
            }
            LCD_text(komunikat,napis);
        }
        else if (gsp != 0)
                gs1 = 0;


        if (alarm)
        {
            PORTD |= (LCD_bckl); // lcd on
            if (gsp != gs2) // co 0,5s
            {
                gs2=gsp;
                PORTD ^= ((1<<BUZZ_pin)|(1<<LED_pin)); // mrugamy
            }
        }
        else if (p1==0)
        {
            PORTD &= ~((1<<BUZZ_pin)|(1<<LED_pin));
        }

    }
    return (0);
}


//// Definicje fcji

void LCD_on()
{
    LCD_ddr |= 0b00111111; //linie na wyjscia
    LCD_port &= 0b11000000; //wszystko na niski stan
    _delay_ms(45);
    LCD_char(0b00101000); //4 linie, wiersze,znak5x8 (chyba ;p)
    LCD_port |= _BV(LCD_rs);
    LCD_port &= ~(_BV(LCD_rs)); //początek komendy
    LCD_char(0b00000110); //znak przesuwamy
    LCD_port |= _BV(LCD_rs);
    LCD_port &= ~(_BV(LCD_rs)); //początek komendy
    LCD_char(0b00001100); //kursor
    LCD_port |= _BV(LCD_rs);
    LCD_clr();
}
void LCD_char(int8_t b)
{
    LCD_port |= _BV(LCD_en); //1 na en :)
    LCD_port = ((b & 0xF0)>>2)|(LCD_port & 0b11000011); //4 starsze do LCD
    LCD_port &= ~(_BV(LCD_en)); //potwierdzam wysłanie - stan niski na en
    asm volatile("nop");
    LCD_port |= _BV(LCD_en);
    LCD_port = ((b & 0x0F)<<2)|(LCD_port & 0b11000011); //4 mlodsze
    LCD_port &= ~(_BV(LCD_en));
    _delay_us(40); //potwierdzenie
}

void LCD_clr()
{
    LCD_port &= ~(_BV(LCD_rs)); //początek komendy
    LCD_char(0b00000001); //czysci ekran
    LCD_port |= _BV(LCD_rs);
    _delay_ms(1.64); //oczekiwanie na czyszczenie
}

void LCD_kom(int kom)
{
    LCD_port &= ~(_BV(LCD_rs)); //początek komendy
    LCD_char(kom); //komenda
    LCD_port |= _BV(LCD_rs);
    _delay_us(1000);
}
void LCD_text(char *txt, char *txt1)
{
    LCD_kom(0x80);
    for (int8_t i=0; i<40; i++)
        LCD_char(txt[i]);
    LCD_kom(0xC0);
    for (int8_t i=0; i<40; i++)
        LCD_char(txt1[i]);
}
void LCD_line(char *txt)
{
    for (int8_t i=0; i<40;i++)
        LCD_char(txt[i]);
}

void ADC_init() // iniciowanie ADC
{
    ADMUX = (1<<REFS0); //Vref = Avcc, muxy 3..0 = 0000 (ADC0)
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //prescaler 128 (16MHz /128 = 125k)- mieści się 50-200kHz
    ADCSRA |= (1<<ADEN); //adc on
}

void TIMER1_init() // inicjowanie timer1 (16bit)
{
    TCCR1B |= (1<<WGM12); // tryb CTC
    TCCR1B |= (1<<CS12)|(1<<CS10); // preskaler na 1024
    OCR1A = 625; //co 25* /  s ((16kk / 1024) / 625) == 25
    TIMSK1 |= (1<<OCIE1A); // zezwolenie na przerwanie wew.
    //TIFR2 =0; // kasowanie flag przerwań
}

ISR(TIMER1_COMPA_vect) // obsługa przerwania (licznik czasu)
{
    if (gsp++ == 25)
        {
            gsp=0;
            if (++gs == 60)
            {
                gs = 0;
                if (++gm==60)
                {
                    gm=0;
                    if (++gh==24) gh=0;
                }
            }
        }
}

int16_t menu_alm_off(int16_t w)
{
    PORTD |= (LCD_bckl); // lcd on
    uint8_t gs1=0, t=0,sw_lock=0;
    char kom1[40] = "Czas wylaczenia alarmu:";
    char kom2[40];
    sw_lock = 240;
    while(1)
    {
        if (w == -1) sprintf (kom2, " wylaczony           ");
        else if (w == 0) sprintf(kom2," do odwolania      ");
        else sprintf(kom2,"  %i sekund       ",w);
        LCD_text(kom1,kom2);
        if (!sw_lock && !(PIND & SW1)) // sprawdzam przycisk
            {
                if (w && w < 30) if (w == -1) w = 5; else w+=5;
                else if (w >= 30) w+=30;
                if (w == 0 ) w = -1;
                if (w > 300) w = 0;
                sw_lock=240;
                t=0;
            }
            else if (sw_lock && (PIND & SW1)) sw_lock++; // zapobiegaganie wielokrotnemu czytaniu + eleiminacja drgan

        if ((PIND & SW1) && gs1 != gs)
        {
            gs1=gs;
            t++;
            if (t==3) // po 3 s
            {
                sprintf(kom2,"   OK !            ");
                LCD_text(kom1,kom2);
                eeprom_update_word(&alarm_off,2*w);

                _delay_ms(1500);
                return (w);
            }
        }
    }

}
//
//float mq2_calculate_ppm( float rs_ro_ratio )
//{
////    float           pcurve[3]  =  {2.3,0.21,-0.47};
//    static const auto ppm_max = 10000.f;
//    static const auto ppm_min = 35.f;
//    static const auto smoke_c1 = 4056.61f;
//    static constexpr auto smoke_c2 = -2.22911f;
//    auto ppm = smoke_c1 * pow( rs_ro_ratio, smoke_c2 );
//    //!Cutoff the potential ivalid ranges
//    if( ppm > ppm_max ) ppm = ppm_max;
//    else if( ppm < ppm_min ) ppm = 0.f;
//    return ppm;
////  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
//}
