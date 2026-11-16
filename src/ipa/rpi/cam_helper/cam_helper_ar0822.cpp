/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 * Copyright (C) 2025-2026, UAB Kurokesu
 *
 * camera helper for ar0822 sensor
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "cam_helper.h"
#include "md_parser.h"

using namespace RPiController;
using namespace libcamera;

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
	ipa::Pwl decompandCurve(const CameraMode &mode) const override;
	std::pair<uint32_t, int32_t> getBlanking(utils::Duration &exposure,
						 utils::Duration minFrameDuration,
						 utils::Duration maxFrameDuration) const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;

	/* Sensor HDR exposure limits. */
	static constexpr int hdrExposureRatio = 16;
	static constexpr int hdrExposureMargin = 35;
	static constexpr int hdrExposureMax = 144 * hdrExposureRatio;

	/* Data pedestal, added before companding. */
	static constexpr double ccmpBlackLevel = 168;

	/* 16-bit to 12-bit compander knees as (16-bit input, 12-bit output). */
	static constexpr std::array<std::pair<double, double>, 7> ccmpKnees = { {
		{ 0, 0 },
		{ 1024, 1024 },
		{ 2048, 1536 },
		{ 4096, 2048 },
		{ 8192, 2560 },
		{ 16384, 3072 },
		{ 32768, 3584 },
	} };
	static constexpr double ccmpWhite = 4095;
	static constexpr double ccmpWhiteSignal = 32768 + (ccmpWhite - 3584) * 64;

	int hdrExposureLimit(int frameLength) const;
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

ipa::Pwl CamHelperAr0822::decompandCurve(const CameraMode &mode) const
{
	if (!mode.sensorHdr || mode.bitdepth != 12)
		return {};

	/* Scale so white code reaches full scale, black level stays at 2688. */
	constexpr double scale = 16;
	constexpr double blackOut = ccmpBlackLevel * scale;
	constexpr double gain = (65535 - blackOut) / (ccmpWhiteSignal - ccmpBlackLevel);

	ipa::Pwl curve;
	for (const auto &[signal, code] : ccmpKnees)
		curve.append(code * scale, blackOut + (signal - ccmpBlackLevel) * gain);
	curve.append(ccmpWhite * scale, 65535);
	curve.append(4096 * scale, 65535);

	return curve;
}

/* Longest T1 that fits the frame in sensor HDR mode. */
int CamHelperAr0822::hdrExposureLimit(int frameLength) const
{
	int t1t2Max = frameLength - hdrExposureMargin;
	int t2Max = t1t2Max - static_cast<int>(mode_.height);
	int t1 = std::min({ t1t2Max * hdrExposureRatio / (hdrExposureRatio + 1),
			    t2Max * hdrExposureRatio, hdrExposureMax });

	return std::max(t1, 1);
}

std::pair<uint32_t, int32_t>
CamHelperAr0822::getBlanking(utils::Duration &exposure, utils::Duration minFrameDuration,
			     utils::Duration maxFrameDuration) const
{
	if (!mode_.sensorHdr)
		return CamHelper::getBlanking(exposure, minFrameDuration, maxFrameDuration);

	utils::Duration lineLength = mode_.minLineLength;
	int frameLengthMin = minFrameDuration / lineLength;
	int frameLengthMax = maxFrameDuration / lineLength;
	int t1 = std::min<uint32_t>(exposureLines(exposure, lineLength), hdrExposureMax);
	int frameLength = std::max<int>(t1, mode_.height) + t1 / hdrExposureRatio +
			  hdrExposureMargin;

	frameLength = std::clamp(frameLength, frameLengthMin, frameLengthMax);
	t1 = std::min(t1, hdrExposureLimit(frameLength));
	exposure = CamHelper::exposure(t1, lineLength);

	return { frameLength - mode_.height, lineLengthToHblank(lineLength) };
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
