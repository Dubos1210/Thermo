/* USART Library for Atmel AVR v0.4
 *  USART_init(9600);           //Инициализация USART на скорости 9600 бод
 *  USART_send(0x**);           //Отправка байта 0x**
 *  USART_send_BK();            //Отправка сигнала "Возврат каретки" (0x0D 0x0A)
 *  USART_send_string("Hello"); //Отправка строки Hello
 *  USART_send_integer(123);    //Отправка числа 0123 в виде ASCII-символов (макс. - 9999)
*/

#ifndef F_CPU
  #define F_CPU 8000000UL
#endif

void USART_init(unsigned int baud) {
	DDRD &=~ (1<<0);
	DDRD |= (1<<1);
	
  unsigned int myubrr = F_CPU / 16 / baud - 1;
  UBRRH = (unsigned char) (myubrr >> 8);
  UBRRL = (unsigned char) myubrr;
  UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
  UCSRC = (0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0);
}

void USART_send(char s) {
  while(!(UCSRA & (1<<UDRE))) {};
  UDR = s;
}

void USART_send_BK(void) {
  USART_send(0x0D);
  USART_send(0x0A);
}

void USART_send_string(const char *data) {
  unsigned char c;
  while(( c = *data++ )) {
    USART_send(c);
    _delay_ms(10);
  }
}

void USART_send_integer(unsigned int c) {
  c = c % 10000;
  unsigned char temp = c / 100;
  if(c >= 1000) { USART_send(0x30 + temp / 10); }
  if(c >= 100) { USART_send(0x30 + temp % 10); }
  temp = c % 100;
  if(c >= 10) { USART_send(0x30 + temp / 10); }
  USART_send(0x30 + temp % 10);
}

void USART_send_long(int32_t c) {
	unsigned long temp = 0;
	if(c < 0) { USART_send(0x2D); }
	c %= 10000000000;
	if(c < 0) { c *= -1;}
	if(c >= 100000000) {
		temp = c / 100000000;
		USART_send(0x30 + temp / 10);
		USART_send(0x30 + temp % 10);
	}
	if(c >= 1000000) {
		temp = (c % 100000000) / 1000000;
		USART_send(0x30 + temp / 10);
		USART_send(0x30 + temp % 10);
	}
	if(c >= 10000) {
		temp = (c % 1000000) / 10000;
		USART_send(0x30 + temp / 10);
		USART_send(0x30 + temp % 10);
	}
	if(c >= 100) {
		temp = (c % 10000) / 100;
		USART_send(0x30 + temp / 10);
		USART_send(0x30 + temp % 10);
	}
	temp = c % 100;
	USART_send(0x30 + temp / 10);
	USART_send(0x30 + temp % 10);
}

void USART_send_decimal(int c, int d) {
	switch(d) {
		case 1:
			USART_send_integer((int) c / 10);
			USART_send(0x2E);
			USART_send_integer((int) c % 10);
			break;
		case 2:
			USART_send_integer((int) c / 100);
			USART_send(0x2E);
			USART_send_integer((int) c % 100);
			break;
		case 3:
			USART_send_integer((int) c / 1000);
			USART_send(0x2E);
			USART_send_integer((int) c % 1000);
			break;
		default:
			USART_send_integer(c);
			break;
	}	
}
