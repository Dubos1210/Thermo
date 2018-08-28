/*
 * Thermo.c
 *
 * Created: 10.06.2018 15:36:04
 * Author : Dubos
 * 
 * avrdude.exe -p m8 -c usbasp -P usb -U flash:w:"G:\_PROJECTS\Thermo\firmware\Thermo\USART_EN\Thermo.hex":a
 */ 

#define F_CPU 12000000UL

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "settings.h"

#include "usart.h"
#include "DS18B20.h"

int temperature = 0;			//Температура
int i, a;						//Счетчики циклов
int NOTcounter = 1;				//Счетчик нажатий кнопки "НЕ НАЖИМАТЬ"
unsigned int counter = 0;		//Счетчик измерений (только для UART)
unsigned char usartbuff[24];

#define DELAY			1		//Половина периода SCK

#define SPI_PORT		PORTC	//PORT регистр
#define SPI_DDR			DDRC	//DDR регистр

#define SPI_MOSI		3		//Вывод MOSI
#define SPI_SCK			4		//Вывод SCK
#define SPI_CS			5		//Вывод CS

#define LED_ON()		PORTB |= (1<<0)
#define LED_OFF()		PORTB &=~ (1<<0)

#define SPI_SelectChip()		SPI_PORT &=~ (1<<SPI_CS)	//Low level to CS
#define SPI_DeselectChip()		SPI_PORT |= (1<<SPI_CS)		//High level to CS

//Send bit to shift registers
void SPI_bitTx(uint8_t data) {
	if(data > 0) {
		SPI_PORT |= (1<<SPI_MOSI);
	}
	else {
		SPI_PORT &=~ (1<<SPI_MOSI);
	}
	data = data << 1;
	_delay_us(DELAY);
	SPI_PORT |= (1<<SPI_SCK);
	_delay_us(DELAY);
	SPI_PORT &=~ (1<<SPI_SCK);
}

//Send value to scale
//type: 0 - number, 1 - positive number, 2 - negative number
void scale(unsigned int number, unsigned char type) {
	SPI_SelectChip();
	switch(type) {
		case 1:
			for(i = 0; i < LEDs-(26+number); i++) {
				SPI_bitTx(0);
			}
			for(i = 0; i <= number; i++) {
				SPI_bitTx(1);
			}
			for(i = 0; i < 26; i++) {
				SPI_bitTx(0);
			}
			break;
		case 2:
			for(i = 0; i < 26; i++) {
				SPI_bitTx(0);
			}			
			for(i = 0; i <= number; i++) {
				SPI_bitTx(1);
			}
			for(i = 0; i < LEDs-(27+number); i++) {
				SPI_bitTx(0);
			}
			break;
		default:
			for(i = 0; i < LEDs-number; i++) {
				SPI_bitTx(0);
			}
			for(i = 0; i < number; i++) {
				SPI_bitTx(1);
			}
			break;
	}
	
	SPI_DeselectChip();
}

//Number interrupt
ISR(INT1_vect) {
	GICR &=~ (1<<INT1);
	
	temperature = DS18B20_temperature();
	temperature /= 2;
	
	NOTcounter++;
	
	scale(temperature, 1);
	
	if(temperature >= 0) {
		for(a = 1; a <= temperature; a++) {
			scale(temperature - a, 1);
			_delay_ms(str_delay);
		}
	}
	else {
		for(a = 1; a <= (-1 * temperature); a++) {
			scale((-1 * temperature) - a, 2);
			_delay_ms(str_delay);
		}		
	}
	
	for(a = 0; a <= NOTcounter; a++) {
		scale(a, 0);
		_delay_ms(str_delay);
	}
	
	_delay_ms(2000);
	
	for(a = 1; a <= NOTcounter; a++) {
		scale(NOTcounter - a, 0);
		_delay_ms(str_delay);
	}
	
	if(temperature >= 0) {
		for(a = 0; a <= temperature; a++) {
			scale(a, 1);
			_delay_ms(str_delay);
		}
	}
	else {
		for(a = 1; a <= (-1 * temperature); a++) {
			scale(a, 2);
			_delay_ms(str_delay);
		}
	}
		
	GIFR |= (1<<INTF1);
	GICR |= (1<<INT1);
}

ISR(ADC_vect) {	
	#if PWM_EN
		OCR1B = ADCL + (ADCH<<8);		
	#endif
		
	ADCSRA |= (1<<ADIF);
}

int main(void)
{
	//Deny interrupts
	cli();
	
	//LED
	DDRB |= (1<<0);
	PORTB |= (1<<0);
		
	//DEBUG LED
	DDRB |= (1<<1);
	PORTB &=~ (1<<1);
		
	//Button interrupt
	DDRD |= (1<<3);
	PORTD |= (1<<3);
	_delay_ms(500);
	if(!(PIND & (1<<3))) {
		//Double blink
		PORTB &=~ (1<<0);
		_delay_ms(50);
		PORTB |= (1<<0);
		_delay_ms(50);
		PORTB &=~ (1<<0);
		_delay_ms(50);
		PORTB |= (1<<0);
		_delay_ms(50);
		PORTB &=~ (1<<0);
		_delay_ms(250);
		PORTB |= (1<<0);
		
		//Interrupt setup
		MCUCR = (1<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00);
		GICR = (1<<INT1)|(0<<INT0);
		GIFR = (1<<INTF1)|(1<<INTF0);
	}
	
	//Set PWM out
	DDRB |= (1<<2);
	PORTB |= (1<<2);
	
	#if PWM_EN	
		//Timer1 setup
		PORTB |= (1<<1);
		TCCR1A = (0<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(1<<COM1B0)|(0<<FOC1A)|(0<<FOC1B)|(1<<WGM11)|(1<<WGM10);
		TCCR1B = (0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);
		OCR1B = 0x03FF;
		//OCR1B = 0x008F;
	
		//ADC setup	
		ADMUX = (1<<REFS1)|(1<<REFS0)|(0<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
		ADCSRA = (1<<ADEN)|(1<<ADSC)|(0<<ADFR)|(1<<ADIF)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(0<ADPS0);	#endif
	#if USART_EN
		USART_init(38400);
		USART_send_string("  /////////////  //"); USART_send_BK();
		USART_send_string(" /////////////  //"); USART_send_BK();
		USART_send_string("     ////      // ////     /////    // ///    // ///  ///    ////"); USART_send_BK();
		USART_send_string("    ////      ///    //  //    //  ///   //  ///   //   //  //  //"); USART_send_BK();
		USART_send_string("   ////      //     //  ///////   //        //    //   //  //   //"); USART_send_BK();
		USART_send_string("  ////      //     //  //        //        //    //   //   //  //"); USART_send_BK();
		USART_send_string(" ////      //     //    /////   //        //    //   //     ////"); USART_send_BK();
		USART_send_string("Dubos Thermometer"); USART_send_BK();
		USART_send_string("(c) 2018, Vladimir Dubos"); USART_send_BK();
		USART_send_string("software.dubos@yandex.ru"); USART_send_BK();
		USART_send_string("dubos1210@yandex.ru  http://www.dubos.ru"); USART_send_BK(); USART_send_BK();
	#endif
		
	//SPI ports setup
	SPI_DDR |= (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_CS);	
	SPI_PORT |= (1<<SPI_CS)|(1<<SPI_MOSI);
	SPI_PORT &=~ (1<<SPI_SCK);
	SPI_bitTx(1);
		
	#if USART_EN
		USART_send_string("Test LEDs");
		USART_send_BK();
	#endif
			
	scale(0, 0);
	_delay_ms(500);
	for(a = 0; a <= LEDs; a++) {
		scale(a, 0);
		_delay_ms(str_delay);
	}

	
	//DS18B20 init
	#if USART_EN
		USART_send_string("Seraching for DS18B20 (Library v0.1 by Dubos)");
	#endif
	if(DS18B20_rst() > 0) {
		#if USART_EN
			USART_send_string(" - Found"); USART_send_BK();
		#endif
		DS18B20_write(SKIP_ROM);
		DS18B20_write(WRITE_SCRATCHPAD);
		DS18B20_write(0x1F);
		DS18B20_write(0x1F);
		DS18B20_write(0x1F);
		temperature = DS18B20_temperature();
		#if USART_EN
			if(temperature >= 0) {
				sprintf(usartbuff, "[%d] Temperature: %d C", counter, temperature);
			}
			else {
				sprintf(usartbuff, "[%d] Temperature: -%d C", counter, -1 * temperature);
			}
			USART_send_string(usartbuff);
			USART_send_BK();
		#endif
	}
	else {
		LED_OFF();
		
		#if USART_EN
			USART_send_string(" - Not found");
			USART_send_BK();
		#endif
	}
	
	temperature /= 2;
	//temperature = -15;
	
	if(temperature >= 0) {
		for(a = 0; a <= (LEDs-((temperature/20) + 26)); a++) {
			scale(LEDs-a, 0);
			_delay_ms(str_delay2);
		}
		scale(temperature, 1);
	}
	else {
		PORTB &=~ (1<<0);
		for(a = 0; a < (LEDs / 2); a++) {
			scale(LEDs-a, 0);
			_delay_ms(str_delay2);
		}
		scale((temperature * (-1)), 2);
	}
	
	//Allow interrupt
	sei();
	LED_OFF();
			
    while (1) 
    {
		if(DS18B20_rst > 0) {
			
			//Temperature measurement
			temperature = DS18B20_temperature();
							
			LED_ON();
		
			//Show temperature		
			SPI_SelectChip();
			if(temperature >= 0) {
				scale((temperature / 2), 1);
				#if USART_EN
					sprintf(usartbuff, "[%d] Temperature: %d C", counter, temperature);
				#endif
			}
			else {
				scale((temperature  / (-2)), 2);
				#if USART_EN
					sprintf(usartbuff, "[%d] Temperature: -%d C", counter, -1 * temperature);
				#endif
			}
			SPI_DeselectChip();	
		
			#if USART_EN
				USART_send_string(usartbuff);
				USART_send_BK();
			#endif	
		
			#if PWM_EN
				//Light measurement
				ADCSRA |= (1<<ADSC);
			#endif	
			
		}
		else {
			scale(0, 0);
		}
			
		LED_OFF();
		
		_delay_ms(upd_delay);
    }
	
}

