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

constexpr double temperatureSlope = 0.71;
constexpr double temperatureCalibration = 60.0;
constexpr uint16_t temperatureMask = 0x3FF;
constexpr uint16_t temperatureCalibVal = 468; /* As 0x30C6 is not reported use hardcoded value */

constexpr uint16_t regExposure = 0x3012;
constexpr uint16_t regAnalogGain = 0x5900;
constexpr uint16_t regFrameLength = 0x300A;
constexpr uint16_t regLineLengthPck = 0x300C;
constexpr uint16_t regTempSens = 0x30B2;
constexpr uint16_t regTempSensCalib1 = 0x30C6; // Contains temperature reading value at 60°C

constexpr std::initializer_list<uint16_t> registerList = {
	regExposure, regAnalogGain, regFrameLength, regLineLengthPck, regTempSens,
	// regTempSensCalib1, For some reason this register is not reported
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
	: CamHelper(std::make_unique<MdParserOnsemi>(registerList), frameIntegrationDiff)
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

	LOG(IPARPI, Debug) << "raw gain " << registers.at(regAnalogGain);
	LOG(IPARPI, Debug) << "raw exposure " << registers.at(regExposure);
	LOG(IPARPI, Debug) << "raw frame length " << registers.at(regFrameLength);
	LOG(IPARPI, Debug) << "raw line length " << registers.at(regLineLengthPck);
	LOG(IPARPI, Debug) << "raw sensor temperature " << registers.at(regTempSens);
	// LOG(IPARPI, Debug) << "raw sensor cal temperature " << registers.at(regTempSensCalib1);

	deviceStatus.lineLength = lineLengthPckToDuration(registers.at(regLineLengthPck));
	deviceStatus.exposureTime = exposure(registers.at(regExposure),
					     deviceStatus.lineLength);
	deviceStatus.analogueGain = gain(registers.at(regAnalogGain));
	deviceStatus.frameLength = registers.at(regFrameLength);
	int tempVal = registers.at(regTempSens) & temperatureMask;
	deviceStatus.sensorTemperature = temperatureSlope * (tempVal - temperatureCalibVal) + temperatureCalibration;

	LOG(IPARPI, Debug) << "device status" << deviceStatus;

	metadata.set("device.status", deviceStatus);
}

static CamHelper *create()
{
	return new CamHelperAr0822();
}

static RegisterCamHelper reg("ar0822", &create);
