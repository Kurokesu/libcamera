/* SPDX-License-Identifier: BSD-2-Clause */

#include "decompand.h"

#include <libcamera/base/log.h>

#include "../decompand_status.h"
#include "../histogram.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiDecompand)

#define NAME "rpi.decompand"

Decompand::Decompand(Controller *controller)
	: DecompandAlgorithm(controller)
{
}

char const *Decompand::name() const
{
	return NAME;
}

int Decompand::read(const libcamera::ValueNode &params)
{
	config_.bitdepth = params["bitdepth"].get<uint32_t>(0);

	if (params.contains("decompand_curve")) {
		config_.decompandCurve = params["decompand_curve"].get<ipa::Pwl>(ipa::Pwl{});
		if (config_.decompandCurve.empty())
			return -EINVAL;
	}

	return 0;
}

void Decompand::initialise()
{
}

void Decompand::switchMode(CameraMode const &cameraMode,
			   [[maybe_unused]] Metadata *metadata)
{
	/* A curve from camera helper takes precedence over the tuning file. */
	if (!cameraMode.decompandCurve.empty())
		curve_ = cameraMode.decompandCurve;
	else if (config_.bitdepth == 0 || cameraMode.bitdepth == config_.bitdepth)
		curve_ = config_.decompandCurve;
	else
		curve_ = {};
}

void Decompand::initialValues(libcamera::ipa::Pwl &decompandCurve)
{
	decompandCurve = curve_;
}

void Decompand::prepare(Metadata *imageMetadata)
{
	DecompandStatus decompandStatus;

	if (!curve_.empty()) {
		decompandStatus.decompandCurve = curve_;
		imageMetadata->set("decompand.status", decompandStatus);
	}
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return new Decompand(controller);
}

static RegisterAlgorithm reg(NAME, &create);
