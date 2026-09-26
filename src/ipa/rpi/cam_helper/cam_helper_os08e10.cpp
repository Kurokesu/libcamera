/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 * Copyright (C) 2026, UAB Kurokesu
 *
 * camera helper for os08e10 sensor
 */

#include "cam_helper.h"

using namespace RPiController;

class CamHelperOs08e10 : public CamHelper
{
public:
	CamHelperOs08e10();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 33;
};

CamHelperOs08e10::CamHelperOs08e10()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperOs08e10::gainCode(double gain) const
{
	return static_cast<uint32_t>(gain * 16.0);
}

double CamHelperOs08e10::gain(uint32_t gainCode) const
{
	return static_cast<double>(gainCode) / 16.0;
}

static CamHelper *create()
{
	return new CamHelperOs08e10();
}

static RegisterCamHelper reg("os08e10", &create);
