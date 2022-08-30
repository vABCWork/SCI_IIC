extern	uint8_t sci_iic_slave_adrs;

void initSCI_12(void);

void sci_rd_sensor_status(void);
void sci_rd_sensor_humi_temp(void);
void sci_wr_sensor_cmd(void);
void sci_cal_humidity_temperature(void);


void sci_rd_thermo_pile( uint32_t rd_obj );

void sci_iic_master_send ( uint8_t sd_num);

void sci_iic_master_rcv ( uint8_t rcv_num);

void sci_iic_sd_start(void);



uint8_t Calc_crc_x8_x5_x4_1(volatile uint8_t *data, uint8_t num);

void sci_iic_cal_crc_thermo_pile(void);

void sci_iic_cal_Ta_To_temperature(void);
