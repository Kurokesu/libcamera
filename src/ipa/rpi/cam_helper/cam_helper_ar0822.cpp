/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025, Kurokesu UAB
 *
 * cam_helper_Ar0822.cpp - camera information for Ar0822 sensor
 */

#include <assert.h>
#include <cmath>
#include <stdlib.h>

#include <libcamera/base/log.h>

#include "cam_helper.h"
#include "md_parser.h"

using namespace RPiController;
using namespace libcamera;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}

#define TEMPERATURE_MASK 0x3FF
#define TEMPERATURE_SLOPE 0.71
#define TEMPERATURE_CALIBRATION 60.0

#define REG_EXPOSURE 0x3012
#define REG_ANALOG_GAIN 0x5900
#define REG_FRAME_LENGTH 0x300A
#define REG_LINE_LENGTH_PCK 0x300C
#define REG_TEMPSENS 0x30B2
#define REG_TEMPSENS_CALIB1 0x30C6 // Contains temperature reading value at 60°C

constexpr std::initializer_list<uint16_t> registerList = {
	REG_EXPOSURE,
	REG_ANALOG_GAIN,
	REG_FRAME_LENGTH,
	REG_LINE_LENGTH_PCK,
	REG_TEMPSENS,
	// REG_TEMPSENS_CALIB1, For some reason this register is not reported
};

class CamHelperAr0822 : public CamHelper
{
public:
	CamHelperAr0822();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
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

CamHelperAr0822::CamHelperAr0822()
	: CamHelper(std::make_unique<MdParserOnSemi>(&registerList), frameIntegrationDiff)
{
	parser_->setNumLines(4);
}

uint32_t CamHelperAr0822::gainCode(double gain) const
{
	int code = std::log10(gain) * (20.0 / 0.375);
	return std::max(0, std::min(code, 119));
}

double CamHelperAr0822::gain(uint32_t gainCode) const
{
	return std::pow(10, gainCode * 0.375 / 20.0);
}

bool CamHelperAr0822::sensorEmbeddedDataPresent() const
{
	return true;
}

void CamHelperAr0822::populateMetadata(const MdParser::RegisterMap &registers,
				       Metadata &metadata) const
{
	DeviceStatus deviceStatus;

	LOG(IPARPI, Debug) << "raw gain " << registers.at(REG_ANALOG_GAIN);
	LOG(IPARPI, Debug) << "raw exposure " << registers.at(REG_EXPOSURE);
	LOG(IPARPI, Debug) << "raw frame length " << registers.at(REG_FRAME_LENGTH);
	LOG(IPARPI, Debug) << "raw line length " << registers.at(REG_LINE_LENGTH_PCK);
	LOG(IPARPI, Debug) << "raw sensor temperature " << registers.at(REG_TEMPSENS);
	// LOG(IPARPI, Debug) << "raw sensor cal temperature " << registers.at(REG_TEMPSENS_CALIB1);

	deviceStatus.lineLength = lineLengthPckToDuration(registers.at(REG_LINE_LENGTH_PCK));
	deviceStatus.exposureTime = exposure(registers.at(REG_EXPOSURE),
				     deviceStatus.lineLength);
	deviceStatus.analogueGain = gain(registers.at(REG_ANALOG_GAIN));
	deviceStatus.frameLength = registers.at(REG_FRAME_LENGTH);
	deviceStatus.sensorTemperature = TEMPERATURE_SLOPE * (int(registers.at(REG_TEMPSENS) & TEMPERATURE_MASK) - 468) + TEMPERATURE_CALIBRATION;

	LOG(IPARPI, Debug) << "device status" << deviceStatus;

	metadata.set("device.status", deviceStatus);
}

static CamHelper *create()
{
	return new CamHelperAr0822();
}

static RegisterCamHelper reg("ar0822", &create);
