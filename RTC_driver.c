typedef struct{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	bool pm;
	uint8_t wday;
	uint8_t mday;
	uint8_t month;
	uint8_t year;
	
	uint8_t amin;
	uint8_t ahour;
	bool apm;
	bool alarm;
	
	uint8_t stat;
	uint8_t ctrl;
}RTC_info;


void init_i2c(void){
	TWCR = 0;//reset control register
	TWBR = 46;
//	TWSR = (1<<TWPS1) | (1<<TWPS0);
	TWCR = (1 << TWEN);//enable two wire communication
}

void i2c_start(void){
	/*
	clear the interrupt flag
	set the start condition bit
	keep i2c enabled
	*/
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	//wait for transmission to finish -- might not be necessary
	while(!(TWCR & (1<<TWINT)));
}

void i2c_stop(void){
	/*
	clear the interrupt flag
	set the stop condition bit
	keep i2c enabled
	*/
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
	//wait for stop condition to finish
	while(TWCR & (1<<TWSTO));
}

void i2c_write(uint8_t data){
	TWDR = data;
	//keep enabled and clear int bit
	TWCR = (1<<TWEN) | (1<<TWINT);
	while(!(TWCR & (1<<TWINT)));
}


uint8_t i2c_read(bool ack){
	TWCR |= (1 << TWINT) | (ack<<TWEA);
	while(!(TWCR & (1<<TWINT)));
	uint8_t test = TWDR;
	return test;
}

void RTC_write(uint8_t address, uint8_t data){
	i2c_start();
	//7 bit address is shifted left one -- write bit 0
	i2c_write((DS3231_addr<<1) & 0xFE);
	i2c_write(address);
	i2c_write(data);
	i2c_stop();
}

uint8_t RTC_read(uint8_t address){
	i2c_start();
	//7 bit address is shifted left one -- write bit 0
	i2c_write((DS3231_addr<<1) & 0xFE);
	i2c_write(address);
	i2c_stop();
	i2c_start();
	//7 bit address is shifted left one -- read bit 1
	i2c_write((DS3231_addr<<1) | 0x01);
	uint8_t data = i2c_read(false);//bus freezes when ack is sent -- unsure why
	i2c_stop();
	return data;
}

//functions for converting between decimal and register storage for RTC
uint8_t reg2dec(uint8_t reg){
	return ((reg / 16 * 10) + (reg % 16));
}
uint8_t dec2reg(uint8_t dec){
    return ((dec / 10 * 16) + (dec % 10));
}

void RTC_read_time(RTC_info * time){
	time->sec = reg2dec(RTC_read(0x00));
	time->min = reg2dec(RTC_read(0x01));
	time->hour = RTC_read(0x02);
	time->pm = ((time->hour & 0x20)  >> 4);//isolating amx/pm bit
	time->hour = reg2dec(time->hour & 0x1F);
	time->wday = RTC_read(0x03);
	time->mday = reg2dec(RTC_read(0x04));
	time->month = reg2dec(RTC_read(0x05));
	time->year = reg2dec(RTC_read(0x06));
	time->ctrl = RTC_read(0x0E);
	time->stat = RTC_read(0x0F);
	time->ahour = RTC_read(0x09);
	time->apm = ((time->ahour & 0x20)  >> 4);//isolating amx/pm bit
	time->ahour = reg2dec(time->ahour & 0x1F);
	time->amin = reg2dec(RTC_read(0x08));
}

void RTC_update_time(RTC_info * time){
	RTC_write(0x00,dec2reg(time->sec));
	RTC_write(0x01,dec2reg(time->min));
	RTC_write(0x02,(dec2reg(time->hour) | (time->pm<<5) | 0x40));
	RTC_write(0x03,dec2reg(time->wday));
	RTC_write(0x04,dec2reg(time->mday));
	RTC_write(0x05,dec2reg(time->month));
	RTC_write(0x06,dec2reg(time->year));
	RTC_write(0x08,dec2reg(time->amin));
	RTC_write(0x09,(dec2reg(time->ahour) | (time->apm<<5) | 0x40));
	RTC_write(0x0E, time->ctrl);
	RTC_write(0x0F, time->stat);
	RTC_write(0x0A, 0x80);//alarm set on hour and minute match
}
