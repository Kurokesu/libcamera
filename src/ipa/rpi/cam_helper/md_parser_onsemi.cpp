/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025-2026, UAB Kurokesu
 *
 * onsemi specification based embedded data parser
 */

#include <libcamera/base/log.h>

#include "md_parser.h"

using namespace RPiController;
using namespace libcamera;

namespace libcamera {
LOG_DEFINE_CATEGORY(ONSEMI)
LOG_DECLARE_CATEGORY(ONSEMI)
} // namespace libcamera

constexpr uint8_t TagLineStart = 0x0A;
constexpr uint8_t TagAddrMsb = 0xAA;
constexpr uint8_t TagAddrLsb = 0xA5;
constexpr uint8_t TagData = 0x5A;
constexpr uint8_t TagEndOfData = 0x07;

constexpr uint8_t RegDataBytes = 2;
constexpr uint8_t RegPacketBytes = 4;

MdParserOnsemi::MdParserOnsemi(std::initializer_list<uint16_t> const *registerList)
{
	for (auto regAddress : *registerList)
		offsets_[regAddress] = {};
}

MdParser::Status MdParserOnsemi::parse(libcamera::Span<const uint8_t> buffer,
				       RegisterMap &registers)
{
	MdParser::Status ret;

	if (reset_) {
		ASSERT((bitsPerPixel_ == 8) || (bitsPerPixel_ == 10) || (bitsPerPixel_ == 12));

		if (bitsPerPixel_ > 8)
			paddingInterval_ = static_cast<uint8_t>(8 / (bitsPerPixel_ - 8));

		ret = findRegs(buffer);
		if (ret != MdParser::Status::OK)
			return ret;

		reset_ = false;
	}

	registers.clear();
	for (const auto &[regAddress, offset] : offsets_) {
		if (!offset) {
			reset_ = true;
			return NOTFOUND;
		}

		uint16_t registerValue;
		ret = getValue(buffer, offset.value(), &registerValue, TagData, TagData);

		if (ret != OK) {
			reset_ = true;
			return ret;
		}

		registers[regAddress] = registerValue;
	}

	return OK;
}

MdParser::Status MdParserOnsemi::findRegs(libcamera::Span<const uint8_t> buffer)
{
	MdParser::Status ret;
	bool waitForLineStart = true, parse = true;
	uint8_t embeddedLine = 0;
	uint16_t index = 0, currentRegAddr = 0;
	uint16_t indexAmount = buffer.size();
	OffsetMap::iterator it = offsets_.begin();

	if (paddingInterval_ > 0)
		indexAmount -= buffer.size() / (paddingInterval_ + 1);

	while ((index < indexAmount) && (it != offsets_.end()) && parse) {
		uint8_t tag = dataWithoutPadding(buffer, index);

		switch (tag) {
		case TagData:
			if (it->first == currentRegAddr) {
				LOG(ONSEMI, Debug)
					<< "Register 0x" << std::hex << it->first
					<< " found at " << std::dec << index;
				it->second = index;
				it++;
			} else if (it->first < currentRegAddr) {
				LOG(ONSEMI, Error)
					<< "Register 0x" << std::hex << it->first
					<< " not found, abort..." << std::dec;
				parse = false;
			}

			currentRegAddr += RegDataBytes;
			index += RegPacketBytes;
			break;
		case TagAddrMsb:
			ret = getValue(buffer, index, &currentRegAddr, TagAddrMsb, TagAddrLsb);
			if (ret != OK)
				return ret;

			index += RegPacketBytes;
			break;
		case TagLineStart:
			embeddedLine++;
			waitForLineStart = false;
			LOG(ONSEMI, Debug)
				<< "Line " << static_cast<int>(embeddedLine)
				<< " start at " << index;
			index++;
			break;
		case TagEndOfData:
			if (!waitForLineStart) {
				LOG(ONSEMI, Debug)
					<< "End of line " << static_cast<int>(embeddedLine)
					<< " at " << index;

				if (embeddedLine == numLines_) {
					LOG(ONSEMI, Debug) << "All embedded lines parsed. Finish";
					parse = false;
				}
			}

			waitForLineStart = true;
			index++;
			break;
		default:
			LOG(ONSEMI, Error)
				<< "Unexpected tag 0x" << std::hex << static_cast<unsigned int>(tag)
				<< " at " << std::dec << index;
			return ERROR;
		}
	}

	if (it != offsets_.end()) {
		LOG(ONSEMI, Error) << "Couldn't find reg 0x" << std::hex << it->first << std::dec;
		return NOTFOUND;
	}

	return OK;
}

MdParser::Status MdParserOnsemi::getValue(libcamera::Span<const uint8_t> buffer,
					  uint16_t index, uint16_t *value, uint8_t tag0, uint8_t tag1)
{
	if ((dataWithoutPadding(buffer, index) != tag0) ||
	    (dataWithoutPadding(buffer, index + RegDataBytes) != tag1)) {
		LOG(ONSEMI, Error) << "Incorrect register value tags at " << index;
		return ERROR;
	}

	index++;
	*value = (dataWithoutPadding(buffer, index) << 8) |
		 dataWithoutPadding(buffer, index + RegDataBytes);

	return OK;
}

uint8_t MdParserOnsemi::dataWithoutPadding(libcamera::Span<const uint8_t> buffer,
					   uint16_t offset)
{
	if (paddingInterval_ > 0)
		offset += offset / paddingInterval_;

	ASSERT(offset < buffer.size());

	return buffer[offset];
}
