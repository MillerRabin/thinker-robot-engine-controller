#include "VL6180x.h"

VL6180x::VL6180x(i2c_inst_t *i2c, uint8_t addr) :
  i2c(i2c), m_addr(addr) {}


int VL6180x::VL6180xInit(void){
  uint8_t data; //for temp data storage

  data = VL6180x_getRegister(VL6180X_SYSTEM_FRESH_OUT_OF_RESET);
  vTaskDelay(pdMS_TO_TICKS(50));
  if(data != 1) return VL6180x_FAILURE_RESET;

  //Required by datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
  VL6180x_setRegister(0x0207, 0x01);
  VL6180x_setRegister(0x0208, 0x01);
  VL6180x_setRegister(0x0096, 0x00);
  VL6180x_setRegister(0x0097, 0xfd);
  VL6180x_setRegister(0x00e3, 0x00);
  VL6180x_setRegister(0x00e4, 0x04);
  VL6180x_setRegister(0x00e5, 0x02);
  VL6180x_setRegister(0x00e6, 0x01);
  VL6180x_setRegister(0x00e7, 0x03);
  VL6180x_setRegister(0x00f5, 0x02);
  VL6180x_setRegister(0x00d9, 0x05);
  VL6180x_setRegister(0x00db, 0xce);
  VL6180x_setRegister(0x00dc, 0x03);
  VL6180x_setRegister(0x00dd, 0xf8);
  VL6180x_setRegister(0x009f, 0x00);
  VL6180x_setRegister(0x00a3, 0x3c);
  VL6180x_setRegister(0x00b7, 0x00);
  VL6180x_setRegister(0x00bb, 0x3c);
  VL6180x_setRegister(0x00b2, 0x09);
  VL6180x_setRegister(0x00ca, 0x09);  
  VL6180x_setRegister(0x0198, 0x01);
  VL6180x_setRegister(0x01b0, 0x17);
  VL6180x_setRegister(0x01ad, 0x00);
  VL6180x_setRegister(0x00ff, 0x05);
  VL6180x_setRegister(0x0100, 0x05);
  VL6180x_setRegister(0x0199, 0x05);
  VL6180x_setRegister(0x01a6, 0x1b);
  VL6180x_setRegister(0x01ac, 0x3e);
  VL6180x_setRegister(0x01a7, 0x1f);
  VL6180x_setRegister(0x0030, 0x00);

  return 0;
}
VL6180x::~VL6180x(void) {
};

void VL6180x::VL6180xDefautSettings(void){
  //Recommended settings from datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf

  //Enable Interrupts on Conversion Complete (any source)
  VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, (4 << 3)|(4) ); // Set GPIO1 high when sample complete


  VL6180x_setRegister(VL6180X_SYSTEM_MODE_GPIO1, 0x10); // Set GPIO1 high when sample complete
  VL6180x_setRegister(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30); //Set Avg sample period
  VL6180x_setRegister(VL6180X_SYSALS_ANALOGUE_GAIN, 0x46); // Set the ALS gain
  VL6180x_setRegister(VL6180X_SYSRANGE_VHV_REPEAT_RATE, 0xFF); // Set auto calibration period (Max = 255)/(OFF = 0)
  VL6180x_setRegister(VL6180X_SYSALS_INTEGRATION_PERIOD, 0x63); // Set ALS integration time to 100ms
  VL6180x_setRegister(VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01); // perform a single temperature calibration
  //Optional settings from datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
  VL6180x_setRegister(VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09); // Set default ranging inter-measurement period to 100ms
  VL6180x_setRegister(VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x0A); // Set default ALS inter-measurement period to 100ms
  VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24); // Configures interrupt on ‘New Sample Ready threshold event’ 
  //Additional settings defaults from community
  VL6180x_setRegister(VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x32);
  VL6180x_setRegister(VL6180X_SYSRANGE_RANGE_CHECK_ENABLES, 0x10 | 0x01);
  VL6180x_setRegister16bit(VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x7B );
  VL6180x_setRegister16bit(VL6180X_SYSALS_INTEGRATION_PERIOD, 0x64);

  VL6180x_setRegister(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD,0x30);
  VL6180x_setRegister(VL6180X_SYSALS_ANALOGUE_GAIN,0x40);
  VL6180x_setRegister(VL6180X_FIRMWARE_RESULT_SCALER,0x01);
}
void VL6180x::getIdentification(struct VL6180xIdentification *temp){

  temp->idModel =  VL6180x_getRegister(VL6180X_IDENTIFICATION_MODEL_ID);
  temp->idModelRevMajor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODEL_REV_MAJOR);
  temp->idModelRevMinor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODEL_REV_MINOR);
  temp->idModuleRevMajor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODULE_REV_MAJOR);
  temp->idModuleRevMinor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODULE_REV_MINOR);

  temp->idDate = VL6180x_getRegister16bit(VL6180X_IDENTIFICATION_DATE);
  temp->idTime = VL6180x_getRegister16bit(VL6180X_IDENTIFICATION_TIME);
}


uint8_t VL6180x::changeAddress(uint8_t old_address, uint8_t new_address){
  
  //NOTICE:  IT APPEARS THAT CHANGING THE ADDRESS IS NOT STORED IN NON-VOLATILE MEMORY
  // POWER CYCLING THE DEVICE REVERTS ADDRESS BACK TO 0X29
 
  if( old_address == new_address) return old_address;
  if( new_address > 127) return old_address;
   
   VL6180x_setRegister(VL6180X_I2C_SLAVE_DEVICE_ADDRESS, new_address);
   
   return VL6180x_getRegister(VL6180X_I2C_SLAVE_DEVICE_ADDRESS); 
}
  


uint8_t VL6180x::getDistance() {
  uint8_t distance;
  VL6180x_setRegister(VL6180X_SYSRANGE_START, 0x01); //Start Single shot mode
  vTaskDelay(pdMS_TO_TICKS(10));
  distance = VL6180x_getRegister(VL6180X_RESULT_RANGE_VAL);
  VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);
  return distance;
}

float VL6180x::getAmbientLight(vl6180x_als_gain VL6180X_ALS_GAIN)
{
  //First load in Gain we are using, do it everytime incase someone changes it on us.
  //Note: Upper nibble shoudl be set to 0x4 i.e. for ALS gain of 1.0 write 0x46
  VL6180x_setRegister(VL6180X_SYSALS_ANALOGUE_GAIN, (0x40 | VL6180X_ALS_GAIN)); // Set the ALS gain

  //Start ALS Measurement 
  VL6180x_setRegister(VL6180X_SYSALS_START, 0x01);
  
  vTaskDelay(pdMS_TO_TICKS(100));

  VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

  //Retrieve the Raw ALS value from the sensoe
  unsigned int alsRaw = VL6180x_getRegister16bit(VL6180X_RESULT_ALS_VAL);
  
  //Get Integration Period for calculation, we do this everytime incase someone changes it on us.
  unsigned int alsIntegrationPeriodRaw = VL6180x_getRegister16bit(VL6180X_SYSALS_INTEGRATION_PERIOD);
  
  float alsIntegrationPeriod = 100.0 / alsIntegrationPeriodRaw ;

  //Calculate actual LUX from Appnotes

  float alsGain = 0.0;
  
  switch (VL6180X_ALS_GAIN){
    case GAIN_20: alsGain = 20.0; break;
    case GAIN_10: alsGain = 10.32; break;
    case GAIN_5: alsGain = 5.21; break;
    case GAIN_2_5: alsGain = 2.60; break;
    case GAIN_1_67: alsGain = 1.72; break;
    case GAIN_1_25: alsGain = 1.28; break;
    case GAIN_1: alsGain = 1.01; break;
    case GAIN_40: alsGain = 40.0; break;
  }

//Calculate LUX from formula in AppNotes
  
  float alsCalculated = (float)0.32 * ((float)alsRaw / alsGain) * alsIntegrationPeriod;

  return alsCalculated;
}

// --- Private Functions --- //

uint8_t VL6180x::VL6180x_getRegister(uint16_t registerAddr)
{
  uint8_t data;
  uint8_t data_write[2];
  uint8_t data_read[1];
  data_write[0] = (registerAddr >> 8) & 0xFF; //MSB of register address 
  data_write[1] = registerAddr & 0xFF; //LSB of register address 
  
  int i2c_ret = i2c_write_blocking(i2c, m_addr, data_write, 2, true);
  if (i2c_ret == PICO_ERROR_GENERIC) {      
    return STATUS_FAIL;
  }
  i2c_ret = i2c_read_blocking(i2c, m_addr, data_read, 1, false);
  if (i2c_ret == PICO_ERROR_GENERIC) {      
    return STATUS_FAIL;
  }

  //Read Data from selected register
  data=data_read[0];
  return data;
}

uint16_t VL6180x::VL6180x_getRegister16bit(uint16_t registerAddr)
{
  uint8_t data_low;
  uint8_t data_high;
  uint16_t data;

  uint8_t data_write[2];
  uint8_t data_read[2];
  data_write[0] = (registerAddr >> 8) & 0xFF; //MSB of register address 
  data_write[1] = registerAddr & 0xFF; //LSB of register address 
  
  int i2c_ret = i2c_write_blocking(i2c, m_addr, data_write, 2, true);
  if (i2c_ret == PICO_ERROR_GENERIC) {      
    return STATUS_FAIL;
  }
  i2c_ret = i2c_read_blocking(i2c, m_addr, data_read, 2, false);
  if (i2c_ret == PICO_ERROR_GENERIC) {      
    return STATUS_FAIL;
  }
  
  data_high = data_read[0]; //Read Data from selected register
  data_low = data_read[1]; //Read Data from selected register
  data = (data_high << 8)|data_low;

  return data;
}

void VL6180x::VL6180x_setRegister(uint16_t registerAddr, uint8_t data)
{
    uint8_t data_write[3];
    data_write[0] = (registerAddr >> 8) & 0xFF; //MSB of register address 
    data_write[1] = registerAddr & 0xFF; //LSB of register address 
    data_write[2] = data & 0xFF; 
    i2c_write_blocking(i2c, m_addr, data_write, 3, false);
}

void VL6180x::VL6180x_setRegister16bit(uint16_t registerAddr, uint16_t data)
{
    uint8_t data_write[4];
    data_write[0] = (registerAddr >> 8) & 0xFF; //MSB of register address 
    data_write[1] = registerAddr & 0xFF; //LSB of register address 
    data_write[2] = (data >> 8) & 0xFF;
    data_write[3] = data & 0xFF; 
    i2c_write_blocking(i2c, m_addr, data_write, 4, false);
}