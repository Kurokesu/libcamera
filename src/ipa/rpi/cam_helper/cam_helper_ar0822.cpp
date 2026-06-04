/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 * Copyright (C) 2025-2026, UAB Kurokesu
 *
 * camera helper for ar0822 sensor
 */

#include <algorithm>
#include <cmath>

#include "cam_helper.h"
#include "md_parser.h"

using namespace RPiController;

constexpr double temperatureSlope = 0.71;
constexpr double temperatureCalibration = 60.0;
constexpr uint16_t temperatureMask = 0x3FF;
constexpr uint16_t temperatureCalibVal = 468; /* As 0x30C6 is not reported use hardcoded value */

constexpr uint16_t regExposure = 0x3012;
constexpr uint16_t regAnalogGain = 0x5900;
constexpr uint16_t regFrameLength = 0x300A;
constexpr uint16_t regLineLengthPck = 0x300C;
constexpr uint16_t regTempSens = 0x30B2;
constexpr uint16_t regTempSensCalib1 = 0x30C6; /* Contains temperature reading value at 60 degrees C */

constexpr std::initializer_list<uint16_t> registerList = {
	regExposure, regAnalogGain, regFrameLength, regLineLengthPck, regTempSens,
	/* regTempSensCalib1, register missing from embedded data */
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
	/*
	 * Analog gain register codes gain in 0.375 dB steps, where gain_dB = 20 * log10(gain).
	 * Convert and clamp to the maximum register value of 119.
	 */
	int code = std::log10(gain) * (20.0 / 0.375);
	return std::clamp(code, 0, 119);
}

double CamHelperAr0822::gain(uint32_t gainCode) const
{
	return std::pow(10.0, gainCode * 0.375 / 20.0);
}

bool CamHelperAr0822::sensorEmbeddedDataPresent() const
{
	return true;
}

void CamHelperAr0822::populateMetadata(const MdParser::RegisterMap &registers,
				       Metadata &metadata) const
{
	DeviceStatus deviceStatus;
	int tempVal = registers.at(regTempSens) & temperatureMask;

	deviceStatus.lineLength = lineLengthPckToDuration(registers.at(regLineLengthPck));
	deviceStatus.exposureTime = exposure(registers.at(regExposure), deviceStatus.lineLength);
	deviceStatus.analogueGain = gain(registers.at(regAnalogGain));
	deviceStatus.frameLength = registers.at(regFrameLength);
	deviceStatus.sensorTemperature = temperatureSlope * (tempVal - temperatureCalibVal) + temperatureCalibration;

	metadata.set("device.status", deviceStatus);
}

static CamHelper *create()
{
	return new CamHelperAr0822();
}

static RegisterCamHelper reg("ar0822", &create);
