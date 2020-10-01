/******************************************************************************
 *
 * MC3672.h
 *
 *****************************************************************************/

#include "boards.h"       // Needed for Arduino Pin definitions.
#include "nrf_drv_twi.h"  // Need for TWI/I2C driver
#include "nrf_delay.h"    // Needed for nrf delay.

#include "MC3672.h"


#ifdef MC36XX_CFG_BUS_I2C
    #define MC36XX_CFG_I2C_ADDR        (0x4C)
#endif

#define MC36XX_CFG_MODE_DEFAULT                 MC36XX_MODE_STANDBY
#define MC36XX_CFG_SAMPLE_RATE_CWAKE_DEFAULT    MC36XX_CWAKE_SR_DEFAULT_54Hz
#define MC36XX_CFG_SAMPLE_RATE_SNIFF_DEFAULT    MC36XX_SNIFF_SR_105Hz//MC36XX_SNIFF_SR_DEFAULT_7Hz
#define MC36XX_CFG_RANGE_DEFAULT                MC36XX_RANGE_8G
#define MC36XX_CFG_RESOLUTION_DEFAULT           MC36XX_RESOLUTION_14BIT
#define MC36XX_CFG_ORIENTATION_MAP_DEFAULT      ORIENTATION_TOP_RIGHT_UP

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* TWI instance. */
static const nrf_drv_twi_t MC36XX_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);


static short x, y, z;

// Raw Accelerometer data
static MC36XX_acc_t AccRaw;


static uint8_t CfgRange, CfgResolution, CfgFifo, CfgINT;



// Repeated Read Byte(s) from register
static void readRegisters(uint8_t reg, byte *buffer, uint8_t len)
{
    ret_code_t err_code;
     
    err_code = nrf_drv_twi_tx(&MC36XX_twi , MC36XX_CFG_I2C_ADDR, &reg, 1, true);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_rx(&MC36XX_twi , MC36XX_CFG_I2C_ADDR, buffer, len); // read a byte
    APP_ERROR_CHECK(err_code);
}

// Read 8-bit from register
static uint8_t readRegister8(uint8_t reg)
{
    uint8_t value = 0;

    readRegisters(reg, &value, 1);
    return value;
}

// Read register bit
static bool readRegisterBit(uint8_t reg, uint8_t pos)
{
    uint8_t value = 0;
    value = readRegister8(reg);
    return ((value >> pos) & 1);
}

// Read 16-bit from register
static int16_t MC36XX_readRegister16(uint8_t reg)
{
    int16_t value = 0;

    readRegisters(reg, (int8_t*)&value, 2);
    return value;
}




// Write 8-bit to register
static void writeRegister8(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {0};
    data[0] = reg;
    data[1] = value;
 
    ret_code_t err_code = nrf_drv_twi_tx(&MC36XX_twi , MC36XX_CFG_I2C_ADDR, data, sizeof(data), false);
    APP_ERROR_CHECK(err_code);
}

// Write register bit
static void writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = readRegister8(reg);

    if (state)
        value |= (1 << pos);
    else
        value &= ~(1 << pos);

    writeRegister8(reg, value);
}

// Write 16-bit to register
static void writeRegister16(uint8_t reg, int16_t value)
{
    uint8_t data[3];
    data[0] = reg;
    *(int16_t*)&data[1] = value;
 
    ret_code_t err_code = nrf_drv_twi_tx(&MC36XX_twi , MC36XX_CFG_I2C_ADDR, data, sizeof(data), false);
    APP_ERROR_CHECK(err_code);

}



//Initialize the MC36XX sensor and set as the default configuration
bool MC36XX_start(unsigned int scl_pin, unsigned int sda_pin)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_MC36XX_config = {
       .scl                = scl_pin,
       .sda                = sda_pin,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    // If event handler is NULL, blocking mode is enabled.
    err_code = nrf_drv_twi_init(&MC36XX_twi, &twi_MC36XX_config, NULL, NULL);
    if(err_code != NRF_SUCCESS)
    {
      nrf_drv_twi_uninit(&MC36XX_twi);
      err_code = nrf_drv_twi_init(&MC36XX_twi, &twi_MC36XX_config, NULL, NULL);
    }
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&MC36XX_twi);


    //Init Reset
    MC36XX_reset();
    MC36XX_SetMode(MC36XX_MODE_STANDBY);

    //SetWakeAGAIN
    MC36XX_SetWakeAGAIN(MC36XX_GAIN_1X);
    //SetSniffAGAIN
    MC36XX_SetSniffAGAIN(MC36XX_GAIN_1X);

    /* Check I2C connection */
    uint8_t id = readRegister8(MC36XX_REG_PROD);
    if (id != 0x71)
    {
        /* No MC36XX detected ... return false */
//      Serial.println("No MC36XX detected!");
//      Serial.println(id, HEX);
        return false;
    }


    //Range: 8g
    MC36XX_SetRangeCtrl(MC36XX_CFG_RANGE_DEFAULT);
    //Resolution: 14bit
    MC36XX_SetResolutionCtrl(MC36XX_CFG_RESOLUTION_DEFAULT);
    //Sampling Rate: 50Hz by default
    MC36XX_SetCWakeSampleRate(MC36XX_CFG_SAMPLE_RATE_CWAKE_DEFAULT);
	//Sampling Rate: 7Hz by default
    MC36XX_SetSniffSampleRate(MC36XX_CFG_SAMPLE_RATE_SNIFF_DEFAULT);
    //Mode: Active
    MC36XX_SetMode(MC36XX_MODE_CWAKE);

    nrf_delay_ms(50);

    return true;
}

void MC36XX_wake()
{
    //Set mode as wake
    MC36XX_SetMode(MC36XX_MODE_CWAKE);
}

void MC36XX_stop()
{
    //Set mode as Sleep
    MC36XX_SetMode(MC36XX_MODE_STANDBY);
}

//Initial reset
void MC36XX_reset()
{
    writeRegister8(MC36XX_REG_MODE_C, 0x01);
    nrf_delay_ms(10);

    writeRegister8(MC36XX_REG_RESET, 0x40);
    nrf_delay_ms(50);

    writeRegister8(MC36XX_REG_STATUS_2, 0x00);
    nrf_delay_ms(10);
    writeRegister8(MC36XX_REG_PWR_CONTROL, 0x42);
    nrf_delay_ms(10);
    writeRegister8(MC36XX_REG_DMX, 0x01);
    nrf_delay_ms(10);
    writeRegister8(MC36XX_REG_DMY, 0x80);
    nrf_delay_ms(10);
    writeRegister8(MC36XX_REG_INITALIZATION2, 0x00);
    nrf_delay_ms(10);
    writeRegister8(MC36XX_REG_INITALIZATION, 0x00);

    nrf_delay_ms(50);

    uint8_t _bRegIO_C = 0;

    _bRegIO_C = readRegister8(MC36XX_REG_FEATURE_CTL);

    #ifdef MC36XX_CFG_BUS_I2C
        _bRegIO_C &= 0x3F;
        _bRegIO_C |= 0x40;
    #else
        _bRegIO_C &= 0x3F;
        _bRegIO_C |= 0x80;
    #endif

    writeRegister8(MC36XX_REG_FEATURE_CTL, _bRegIO_C);

    nrf_delay_ms(50);

    writeRegister8(MC36XX_REG_MODE_C, 0x01);

    nrf_delay_ms(10);
}

void MC36XX_sniff()
{
    //Set mode as Sleep
    MC36XX_SetMode(MC36XX_MODE_SNIFF);
}

void MC36XX_sniffreset()
{
    uint8_t value;
	
    value = readRegister8(MC36XX_REG_SNIFF_CONF_C);
    value |= 0b10000000;
	
    writeRegister8(MC36XX_REG_SNIFF_CONF_C, value);
}

//Set the operation mode
void MC36XX_SetMode(MC36XX_mode_t mode)
{
    uint8_t value;
    uint8_t cfgfifovdd = 0x42;
	
    value = readRegister8(MC36XX_REG_MODE_C);
    value &= 0b11110000;
    value |= mode;
	
    writeRegister8(MC36XX_REG_PWR_CONTROL, cfgfifovdd);
    writeRegister8(MC36XX_REG_MODE_C, value);
}

//Set the range control
void MC36XX_SetRangeCtrl(MC36XX_range_t range)
{
    uint8_t value;
    CfgRange = range;
    MC36XX_SetMode(MC36XX_MODE_STANDBY);
    value = readRegister8(MC36XX_REG_RANGE_C);
    value &= 0b00000111;
    value |= (range << 4)&0x70 ;
    writeRegister8(MC36XX_REG_RANGE_C, value);
}

//Set the resolution control
void MC36XX_SetResolutionCtrl(MC36XX_resolution_t resolution)
{
    uint8_t value;
    CfgResolution = resolution;
    MC36XX_SetMode(MC36XX_MODE_STANDBY);
    value = readRegister8(MC36XX_REG_RANGE_C);
    value &= 0b01110000;
    value |= resolution;
    writeRegister8(MC36XX_REG_RANGE_C, value);
}

//Set the sampling rate
void MC36XX_SetCWakeSampleRate(MC36XX_cwake_sr_t sample_rate)
{
    uint8_t value;
    MC36XX_SetMode(MC36XX_MODE_STANDBY);
    value = readRegister8(MC36XX_REG_WAKE_C);
    value &= 0b00000000;
    value |= sample_rate;
    writeRegister8(MC36XX_REG_WAKE_C, value);
}

//Set the sniff sampling rate
void MC36XX_SetSniffSampleRate(MC36XX_sniff_sr_t sniff_sr)
{
    uint8_t value;
    MC36XX_SetMode(MC36XX_MODE_STANDBY);
    value = readRegister8(MC36XX_REG_SNIFF_C);
    value &= 0b00000000;
    value |= sniff_sr;
    writeRegister8(MC36XX_REG_SNIFF_C, value);
}

//Set FIFO
void MC36XX_SetFIFOCtrl(MC36XX_fifo_ctl_t fifo_ctl,
                         MC36XX_fifo_mode_t fifo_mode,
						 uint8_t fifo_thr)
{
    if (fifo_thr > 31)	//maximum threshold
        fifo_thr = 31;
		
    MC36XX_SetMode(MC36XX_MODE_STANDBY);
    
    CfgFifo = ((fifo_ctl << 6) | (fifo_mode << 5) | fifo_thr);
    writeRegister8(MC36XX_REG_FIFO_C, CfgFifo);
}

//Set interrupt control register
void MC36XX_SetINTCtrl(uint8_t fifo_thr_int_ctl,
                        uint8_t fifo_full_int_ctl,
						uint8_t fifo_empty_int_ctl,
						uint8_t acq_int_ctl,
						uint8_t wake_int_ctl)
{
		
    MC36XX_SetMode(MC36XX_MODE_STANDBY);
    
    CfgINT = (((fifo_thr_int_ctl & 0x01) << 6)
           | ((fifo_full_int_ctl & 0x01) << 5)
           | ((fifo_empty_int_ctl & 0x01) << 4)
           | ((acq_int_ctl & 0x01) << 3)
           | ((wake_int_ctl & 0x01) << 2)
           | MC36XX_INTR_C_IAH_ACTIVE_HIGH//MC36XX_INTR_C_IAH_ACTIVE_LOW//
           | MC36XX_INTR_C_IPP_MODE_PUSH_PULL);//MC36XX_INTR_C_IPP_MODE_OPEN_DRAIN);//
    writeRegister8(MC36XX_REG_INTR_C, CfgINT);
}

//Interrupt handler (clear interrupt flag)
void MC36XX_INTHandler(MC36XX_interrupt_event_t *ptINT_Event)
{
    uint8_t value;

    value = readRegister8(MC36XX_REG_STATUS_2);

    ptINT_Event->bWAKE           = ((value >> 2) & 0x01);
    ptINT_Event->bACQ            = ((value >> 3) & 0x01);
    ptINT_Event->bFIFO_EMPTY     = ((value >> 4) & 0x01);
    ptINT_Event->bFIFO_FULL      = ((value >> 5) & 0x01);
    ptINT_Event->bFIFO_THRESHOLD = ((value >> 6) & 0x01);
    ptINT_Event->bSWAKE_SNIFF    = ((value >> 7) & 0x01);
	
    value &= 0x03;
    writeRegister8(MC36XX_REG_STATUS_2, value);
}

//Set CWake Analog Gain
void MC36XX_SetWakeAGAIN(MC36XX_gain_t gain)
{
    writeRegister8(0x20, 0x01);
    uint8_t value;
    value = readRegister8(MC36XX_REG_GAIN);
    value &= 0b00111111;
    value |= (gain << 6);
    writeRegister8(MC36XX_REG_GAIN, value);
}

//Set Sniff Analog Gain
void MC36XX_SetSniffAGAIN(MC36XX_gain_t gain)
{
    writeRegister8(0x20, 0x00);
    uint8_t value;
    value = readRegister8(MC36XX_REG_GAIN);
    value &= 0b00111111;
    value |= (gain << 6);
    writeRegister8(MC36XX_REG_GAIN, value);
}

//Set Sniff threshold
void MC36XX_SetSniffThreshold(MC36XX_axis_t axis_cfg, uint8_t sniff_thr)
{
    uint8_t value;
	uint8_t regSniff_addr;
    value = readRegister8(MC36XX_REG_SNIFFTH_C);

    switch(axis_cfg)
    {
    case MC36XX_AXIS_X:
        regSniff_addr = 0x01; //Put X-axis to active
        break;
    case MC36XX_AXIS_Y: //Put Y-axis to active
        regSniff_addr = 0x02;
        break;
    case MC36XX_AXIS_Z: //Put Z-axis to active
        regSniff_addr = 0x03;
        break;
    default:
        break;
    }
	
    writeRegister8(MC36XX_REG_SNIFF_CONF_C, regSniff_addr);
    value |= sniff_thr;
    writeRegister8(MC36XX_REG_SNIFFTH_C, value);
}

//Set Sniff detect counts, 1~62 events
void MC36XX_SetSniffDetectCount(MC36XX_axis_t axis_cfg, uint8_t sniff_cnt)
{
    uint8_t value;
    uint8_t sniff_cfg;
    uint8_t regSniff_addr;
	
    sniff_cfg = readRegister8(MC36XX_REG_SNIFF_CONF_C);
	
    switch(axis_cfg)
    {
    case MC36XX_AXIS_X: //Select x detection count shadow register
        regSniff_addr = 0x05;
        break;
    case MC36XX_AXIS_Y: //Select y detection count shadow register
        regSniff_addr = 0x06;
        break;
    case MC36XX_AXIS_Z: //Select z detection count shadow register
        regSniff_addr = 0x07;
        break;
    default:
        break;
    }
	
    sniff_cfg |= regSniff_addr;
    writeRegister8(MC36XX_REG_SNIFF_CONF_C, sniff_cfg);
	
    value = readRegister8(MC36XX_REG_SNIFFTH_C);
	
    value |= sniff_cnt;
    writeRegister8(MC36XX_REG_SNIFFTH_C, value);
	
    sniff_cfg |= 0x08;
    writeRegister8(MC36XX_REG_SNIFF_CONF_C, sniff_cfg);
}

//Set sensor interrupt mode
void MC36XX_SetSniffAndOrN(MC36XX_andorn_t logicandor)
{
    uint8_t value;
	
    value = readRegister8(MC36XX_REG_SNIFFTH_C);
	
    switch(logicandor)
    {
    case MC36XX_ANDORN_OR:  //Axis or mode
        value &= 0xBF;
        break;
    case MC36XX_ANDORN_AND: //Axis and mode
        value |= 0x40;
        break;
    default:
        break;
    }
	
	writeRegister8(MC36XX_REG_SNIFFTH_C, value);
}

//Set sensor sniff delta mode
void MC36XX_SetSniffDeltaMode(MC36XX_delta_mode_t deltamode)
{
    uint8_t value;
	
    value = readRegister8(MC36XX_REG_SNIFFTH_C);
	
    switch(deltamode)
    {
    case MC36XX_DELTA_MODE_C2P: //Axis C2P mode
        value &= 0x7F;
        break;
    case MC36XX_DELTA_MODE_C2B: //Axis C2B mode
        value |= 0x80;
        break;
    default:
        break;
    }
	
    writeRegister8(MC36XX_REG_SNIFFTH_C, value);
	
    value = readRegister8(MC36XX_REG_SNIFFTH_C);
//  Serial.println("SniffModeSet");
//  Serial.println(value, HEX);
}

//Get the range control
MC36XX_range_t MC36XX_GetRangeCtrl(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = readRegister8(MC36XX_REG_RANGE_C);
//  Serial.println("GetRangeCtrl");
//  Serial.println(value, HEX);
    value &= 0x70;
    return (MC36XX_range_t) (value >> 4);
}

//Get the range control
MC36XX_resolution_t MC36XX_GetResolutionCtrl(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = readRegister8(MC36XX_REG_RANGE_C);
//  Serial.println("GetResolutionCtrl");
//  Serial.println(value, HEX);
    value &= 0x07;
    return (MC36XX_resolution_t) (value);
}

//Get the output sampling rate
MC36XX_cwake_sr_t MC36XX_GetCWakeSampleRate(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = readRegister8(MC36XX_REG_WAKE_C);
//  Serial.println("GetCWakeSampleRate");
//  Serial.println(value, HEX);
    value &= 0b00001111;
    return (MC36XX_cwake_sr_t) (value);
}

//Get the sniff sample rate
MC36XX_sniff_sr_t MC36XX_GetSniffSampleRate(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = readRegister8(MC36XX_REG_SNIFF_C);
//  Serial.println("GetSniffSampleRate");
//  Serial.println(value, HEX);
    value &= 0b00001111;
    return (MC36XX_sniff_sr_t) (value);
}

//Is FIFO empty
bool MC36XX_IsFIFOEmpty(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = readRegister8(MC36XX_REG_STATUS_1);
    value &= 0x10;
//  Serial.println("FIFO_Status");
//  Serial.println(value, HEX);
	
    if (value^0x10)
        return false;	//Not empty
    else
        return true;	//Is empty
}

//Read the raw counts and SI units measurement data
void MC36XX_readRawAccel(MC36XX_acc_t* AccRaw)
{
    //{2g, 4g, 8g, 16g, 12g}
    float faRange[5] = { 19.614f, 39.228f, 78.456f, 156.912f, 117.684f};
    //{6bit, 7bit, 8bit, 10bit, 12bit, 14bit}
    float faResolution[6] = { 32.0f, 64.0f, 128.0f, 512.0f, 2048.0f, 8192.0f};

    byte rawData[6];
    // Read the six raw data registers into data array
    readRegisters(MC36XX_REG_XOUT_LSB, rawData, 6);
    x = (short)((((unsigned short)rawData[1]) << 8) | rawData[0]);
    y = (short)((((unsigned short)rawData[3]) << 8) | rawData[2]);
    z = (short)((((unsigned short)rawData[5]) << 8) | rawData[4]);

    AccRaw->XAxis = (short) (x);
    AccRaw->YAxis = (short) (y);
    AccRaw->ZAxis = (short) (z);
    AccRaw->XAxis_g = (float) (x)/faResolution[CfgResolution]*faRange[CfgRange];
    AccRaw->YAxis_g = (float) (y)/faResolution[CfgResolution]*faRange[CfgRange];
    AccRaw->ZAxis_g = (float) (z)/faResolution[CfgResolution]*faRange[CfgRange];
}