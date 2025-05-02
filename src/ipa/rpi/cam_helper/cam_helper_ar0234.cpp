/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * cam_helper_Ar0234.cpp - camera information for Ar0234 sensor
 */

#include <assert.h>
#include <cmath>

#include "cam_helper.h"
#include "md_parser.h"

using namespace RPiController;
/*
 * We care about two gain registers and a pair of exposure registers. Their
 * I2C addresses from the OnSemi AR0234 datasheet:
 */
#define REG_EXPOSURE 0x3012
#define REG_ANALOG_GAIN 0x3060
#define REG_FRAME_LENGTH 0x300A
#define REG_LINE_LENGTH 0x300C
#define REG_TEMPSENS 0x30B2
#define REG_TEMPSENS_CALIB1 0x30C6 // Contains temperature reading value at 55C

constexpr std::initializer_list<uint16_t> registerList = { REG_EXPOSURE, REG_ANALOG_GAIN, REG_FRAME_LENGTH,
							   REG_LINE_LENGTH, REG_TEMPSENS, REG_TEMPSENS_CALIB1 };

class CamHelperAr0234 : public CamHelper
{
public:
	CamHelperAr0234();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	unsigned int hideFramesStartup() const override;
	unsigned int hideFramesModeSwitch() const override;
	bool sensorEmbeddedDataPresent() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;

	void populateMetadata(const MdParser::RegisterMap &registers,
			      Metadata &metadata) const override;
};

CamHelperAr0234::CamHelperAr0234()
	: CamHelper(std::make_unique<MdParserOnSemi>(&registerList), frameIntegrationDiff)
{
}

uint32_t CamHelperAr0234::gainCode(double gain) const
{
	/* The recommended minimum gain is 1.6842 to avoid artifacts. */
	gain = std::clamp(gain, 1.0 / (1.0 - 13.0 / 32.0), 18.45);

	/*
		* The analogue gain is made of a coarse exponential gain in
		* the range [2^0, 2^4] and a fine inversely linear gain in the
		* range [1.0, 2.0[. There is an additional fixed 1.153125
		* multiplier when the coarse gain reaches 2^2.
		*/

	if (gain > 4.0)
		gain /= 1.153125;

	unsigned int coarse = std::log2(gain);
	unsigned int fine = (1 - (1 << coarse) / gain) * 32;

	/* The fine gain rounding depends on the coarse gain. */
	if (coarse == 1 || coarse == 3)
		fine &= ~1;
	else if (coarse == 4)
		fine &= ~3;

	return (coarse << 4) | (fine & 0xf);
}

double CamHelperAr0234::gain(uint32_t gainCode) const
{
	unsigned int coarse = (gainCode >> 4) & 0x7;
	unsigned int fine = gainCode & 0xf;
	unsigned int d1;
	double d2, m;

	switch (coarse) {
	default:
	case 0:
		d1 = 1;
		d2 = 32.0;
		m = 1.0;
		break;
	case 1:
		d1 = 2;
		d2 = 16.0;
		m = 1.0;
		break;
	case 2:
		d1 = 1;
		d2 = 32.0;
		m = 1.153125;
		break;
	case 3:
		d1 = 2;
		d2 = 16.0;
		m = 1.153125;
		break;
	case 4:
		d1 = 4;
		d2 = 8.0;
		m = 1.153125;
		break;
	}

	/*
		* With infinite precision, the calculated gain would be exact,
		* and the reverse conversion with gainCode() would produce the
		* same gain code. In the real world, rounding errors may cause
		* the calculated gain to be lower by an amount negligible for
		* all purposes, except for the reverse conversion. Converting
		* the gain to a gain code could then return the quantized value
		* just lower than the original gain code. To avoid this, tests
		* showed that adding the machine epsilon to the multiplier m is
		* sufficient.
		*/
	m += std::numeric_limits<decltype(m)>::epsilon();

	return m * (1 << coarse) / (1.0 - (fine / d1) / d2);
}

unsigned int CamHelperAr0234::hideFramesStartup() const
{
	/* On startup, we seem to get 1 bad frame. */
	return 1;
}

unsigned int CamHelperAr0234::hideFramesModeSwitch() const
{
	/* After a mode switch, we seem to get 1 bad frame. */
	return 1;
}

bool CamHelperAr0234::sensorEmbeddedDataPresent() const
{
	return true;
}

void CamHelperAr0234::populateMetadata(const MdParser::RegisterMap &registers,
				       Metadata &metadata) const
{
	DeviceStatus deviceStatus;

	deviceStatus.lineLength = 4.0 * lineLengthPckToDuration(registers.at(REG_LINE_LENGTH));
	deviceStatus.exposureTime = exposure(registers.at(REG_EXPOSURE),
					     deviceStatus.lineLength);
	deviceStatus.analogueGain = gain(registers.at(REG_ANALOG_GAIN));
	deviceStatus.frameLength = registers.at(REG_FRAME_LENGTH);
	deviceStatus.sensorTemperature = 0.7 * (int(registers.at(REG_TEMPSENS) & 0x3ff) - int(registers.at(REG_TEMPSENS_CALIB1) & 0x3ff)) + 55.0;

	metadata.set("device.status", deviceStatus);
}

static CamHelper *create()
{
	return new CamHelperAr0234();
}

static RegisterCamHelper reg("ar0234", &create);
