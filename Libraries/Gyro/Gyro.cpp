#include <Gyro.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define L3G4200D_ADDRESS_SA0_LOW  (0xD0 >> 1)
#define L3G4200D_ADDRESS_SA0_HIGH (0xD2 >> 1)
#define L3GD20_ADDRESS_SA0_LOW    (0xD4 >> 1)
#define L3GD20_ADDRESS_SA0_HIGH   (0xD6 >> 1)

// Public Methods //////////////////////////////////////////////////////////////

Gyro::Gyro(){}
Gyro::Gyro(byte enableValue, byte scaleValue, byte sampleTimeValue)
{
	scale=scaleValue;
	sampleTime=sampleTimeValue;
	uint8_t sampleTime=0;
	uint8_t scale=1;
	int dc_offset=0;
	int rate=0;
	float noise=0;
	int prev_rate;
	float angle=0;
	Gyro::init();
	Gyro::enable(enableValue, ((scale-1)<<4)&0x30);
}


bool Gyro::init(byte device, byte sa0)
{
	Wire.begin();
	_device = device;
	switch (_device)
	{
		case L3G4200D_DEVICE:
			if (sa0 == L3G_SA0_LOW)
			{
				address = L3G4200D_ADDRESS_SA0_LOW;
				return true;
			}
			else if (sa0 == L3G_SA0_HIGH)
			{
				address = L3G4200D_ADDRESS_SA0_HIGH;
				return true;
			}
			else
			return autoDetectAddress();
		break;

		case L3GD20_DEVICE:
		if (sa0 == L3G_SA0_LOW)
		{
			address = L3GD20_ADDRESS_SA0_LOW;
			return true;
		}
		else if (sa0 == L3G_SA0_HIGH)
		{
			address = L3GD20_ADDRESS_SA0_HIGH;
			return true;
		}
		else
		return autoDetectAddress();
		break;

		default:
		return autoDetectAddress();
	}
}

// Turns on the L3G's gyro and places it in normal mode.
void Gyro::enableDefault(void)
{
	// 0x0F = 0b00001111
	// Normal power mode, all axes enabled
	writeReg(L3G_CTRL_REG1, 0x0F);
}
void Gyro::enable(byte enable, byte scale)
{
	writeReg(L3G_CTRL_REG1, enable);
	writeReg(L3G_CTRL_REG4, scale);
}

// Writes a gyro register
void Gyro::writeReg(byte reg, byte value)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

// Reads a gyro register
byte Gyro::readReg(byte reg)
{
	byte value;

	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(address, (byte)1);
	value = Wire.read();
	Wire.endTransmission();

	return value;
}

// Reads the 3 gyro channels and stores them in vector g
void Gyro::read()
{
	Wire.beginTransmission(address);
	// assert the MSB of the address to get the gyro
	// to do slave-transmit subaddress updating.
	Wire.write(L3G_OUT_X_L | (1 << 7));
	Wire.endTransmission();
	Wire.requestFrom(address, (byte)6);

	while (Wire.available() < 6);

	uint8_t xlg = Wire.read();
	uint8_t xhg = Wire.read();
	uint8_t ylg = Wire.read();
	uint8_t yhg = Wire.read();
	uint8_t zlg = Wire.read();
	uint8_t zhg = Wire.read();

	// combine high and low bytes
	g.x = (int16_t)(xhg << 8 | xlg);
	g.y = (int16_t)(yhg << 8 | ylg);
	g.z = (int16_t)(zhg << 8 | zlg);
}

void Gyro::readZ()
{
	Wire.beginTransmission(address);
	Wire.write(L3G_OUT_Z_L| (1 << 7));
	Wire.endTransmission();
	Wire.requestFrom(address, (byte)2);

	while (Wire.available() < 2);

	uint8_t zlg = Wire.read();
	uint8_t zhg = Wire.read();

	// combine high and low bytes
	g.z = (int16_t)(zhg << 8 | zlg);
}

void Gyro::calibrateZ(void)
{
	for(int n=0;n<SAMPLENUM;n++)
	{ 
		this->readZ(); 
		dc_offset+=(int)g.z; 
		delay(5);
	} 
	dc_offset=dc_offset/SAMPLENUM; 
 
/* 	for(int n=0;n<SAMPLENUM;n++)
	{ 
		this->readZ(); 
		if((int)g.z-dc_offset>noise) 
			noise=(int)g.z-dc_offset; 
		else if((int)g.z-dc_offset<-noise) 
			noise=-(int)g.z-dc_offset; 
		delay(5);
	}  */
//	noise=noise/(double)(114.286/scale);
    noise = 2;

	if (noise<1) noise=1;
  	Serial.println(); 
	Serial.print("DC Offset: "); 
	Serial.print(dc_offset); 
	Serial.print("\tNoise Level: "); 
	Serial.print(noise); 
	Serial.println();   
}

// Static Methods //////////////////////////////////////////////////////////////

void Gyro::staticComputeZ(int objectPointer)
{
	Gyro* gyro = (Gyro*) objectPointer;
	gyro->readZ(); 
	gyro->rate=((int)gyro->g.z-gyro->dc_offset)/(114.286/gyro->scale); 
	// Ignore the gyro if our angular velocity does not meet our threshold 
	if(gyro->rate >= gyro->noise || gyro->rate <= -gyro->noise)
	{
		gyro->angle += ((double)(gyro->prev_rate + gyro->rate) * (double)gyro->sampleTime) * 1.124F / 2000UL; 
	}
	// remember the current speed for the next loop rate integration. 
	gyro->prev_rate = gyro->rate; 
	
	// Keep our angle between 0-359 degrees 
/* 	if (gyro->angle < 0) 
		gyro->angle += 360; 
	else if (gyro->angle >= 360) 
		gyro->angle -= 360; */
	} 

void Gyro::vector_cross(const vector *a,const vector *b, vector *out)
{
	out->x = a->y*b->z - a->z*b->y;
	out->y = a->z*b->x - a->x*b->z;
	out->z = a->x*b->y - a->y*b->x;
}

float Gyro::vector_dot(const vector *a,const vector *b)
{
	return a->x*b->x+a->y*b->y+a->z*b->z;
}

void Gyro::vector_normalize(vector *a)
{
	float mag = sqrt(vector_dot(a,a));
	a->x /= mag;
	a->y /= mag;
	a->z /= mag;
}

// Get Methods //////////////////////////////////////////////////////////////
uint8_t Gyro::getSampleTime(void) {return sampleTime;}
uint8_t Gyro::getScale(void) {return scale;}
int     Gyro::getDc_offset(void) {return dc_offset;}
int     Gyro::getRate(void) {return rate;}
double   Gyro::getNoise(void) {return noise;}
float   Gyro::getAngle(void) {return angle;}



// Private Methods //////////////////////////////////////////////////////////////

bool Gyro::autoDetectAddress(void)
{
	// try each possible address and stop if reading WHO_AM_I returns the expected response
	address = L3G4200D_ADDRESS_SA0_LOW;
	if (readReg(L3G_WHO_AM_I) == 0xD3) return true;
	address = L3G4200D_ADDRESS_SA0_HIGH;
	if (readReg(L3G_WHO_AM_I) == 0xD3) return true;
	address = L3GD20_ADDRESS_SA0_LOW;
	if (readReg(L3G_WHO_AM_I) == 0xD4) return true;
	address = L3GD20_ADDRESS_SA0_HIGH;
	if (readReg(L3G_WHO_AM_I) == 0xD4) return true;

	return false;
}