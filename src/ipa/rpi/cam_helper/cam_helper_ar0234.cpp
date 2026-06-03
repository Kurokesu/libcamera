/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 * Copyright (C) 2025-2026, UAB Kurokesu
 *
 * camera helper for ar0234 sensor
 */

#include <assert.h>
#include <cmath>

#include "cam_helper.h"
#include "md_parser.h"

using namespace RPiController;

#define TEMPERATURE_MASK 0x3FF
#define TEMPERATURE_SLOPE 0.7
#define TEMPERATURE_CALIBRATION 55.0

#define REG_EXPOSURE 0x3012
#define REG_ANALOG_GAIN 0x3060
#define REG_FRAME_LENGTH 0x300A
#define REG_LINE_LENGTH_PCK 0x300C
#define REG_TEMPSENS 0x30B2
#define REG_TEMPSENS_CALIB1 0x30C6 // Contains temperature reading value at 55°C

constexpr std::initializer_list<uint16_t> registerList = { REG_EXPOSURE, REG_ANALOG_GAIN, REG_FRAME_LENGTH,
							   REG_LINE_LENGTH_PCK, REG_TEMPSENS, REG_TEMPSENS_CALIB1 };

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
	: CamHelper(std::make_unique<MdParserOnsemi>(&registerList), frameIntegrationDiff)
{
}

uint32_t CamHelperAr0234::gainCode(double gain) const
{
	/*
	 * The analogue gain register (0x3060) has a coarse exponential gain in
	 * the range [2^0, 2^4] and a fine inversely linear gain.
	 *
	 * s = R0x3060[6:4], t = R0x3060[3:0]
	 * coarse gain = 2^s
	 * fine gain =
	 *   - s = 0 or 2: 1 / (1 - (t / 32))
	 *   - s = 1 or 3: 1 / (1 - (INT(t / 2) / 16))
	 *   - s = 4:      1 / (1 - (INT(t / 4) / 8))
	 * total gain = coarse_gain * fine_gain
	 *
	 * The recommended minimum gain is 1.68421 to avoid artifacts. The
	 * recommended gain table tops out at 16.0 (0x0040), so we clamp to 16.0
	 * which in practice means s=4 always uses t=0.
	 */
	gain = std::clamp(gain, 1.0 / (1.0 - 13.0 / 32.0), 16.0);
	uint8_t regCoarse = std::log2(gain);
	uint8_t regFine = 0;

	/*
	 * Invert the gain model.
	 *
	 * We always compute the fine code at 32x precision and truncate toward 0
	 * when converting to uint8_t. For odd s values (1, 3) the developer
	 * guide specifies INT(t/2), so we clear the LSB to match. For s=4 we
	 * keep t=0 as we clamp the gain to <= 16.0.
	 */
	if (regCoarse < 4) {
		double gainCoarse = (1 << regCoarse); /* 2^s */
		regFine = static_cast<uint8_t>((1.0 - (gainCoarse / gain)) * 32.0);
	}

	if (regCoarse % 2 != 0)
		regFine &= ~0x1;

	return (regCoarse << 4) | (regFine & 0xF);
}

double CamHelperAr0234::gain(uint32_t gainCode) const
{
	const uint8_t regCoarse = (gainCode >> 4) & 0x7; /* s */
	const uint8_t regFine = gainCode & 0xF; /* t */
	const double gainCoarse = static_cast<double>(1 << regCoarse); /* 2^s */
	double gainFine = 1.0;

	/*
	 * Add epsilon to avoid rounding producing the quantized value just below
	 * the original gain code when converting back with gainCode().
	 */
	const double m = 1.0 + std::numeric_limits<double>::epsilon();

	if (regCoarse < 4)
		gainFine = 1 / (1 - (regFine / 32.0));

	return m * gainCoarse * gainFine;
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

	deviceStatus.lineLength = lineLengthPckToDuration(registers.at(REG_LINE_LENGTH_PCK));
	deviceStatus.exposureTime = exposure(registers.at(REG_EXPOSURE),
					     deviceStatus.lineLength);
	deviceStatus.analogueGain = gain(registers.at(REG_ANALOG_GAIN));
	deviceStatus.frameLength = registers.at(REG_FRAME_LENGTH);
	deviceStatus.sensorTemperature = TEMPERATURE_SLOPE * (int(registers.at(REG_TEMPSENS) & TEMPERATURE_MASK) 
		- int(registers.at(REG_TEMPSENS_CALIB1) & TEMPERATURE_MASK)) + TEMPERATURE_CALIBRATION;

	metadata.set("device.status", deviceStatus);
}

static CamHelper *create()
{
	return new CamHelperAr0234();
}

static RegisterCamHelper reg("ar0234", &create);
