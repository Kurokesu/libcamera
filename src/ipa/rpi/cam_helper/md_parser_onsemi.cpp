#include <libcamera/base/log.h>

#include "md_parser.h"

using namespace RPiController;
using namespace libcamera;

namespace libcamera {
LOG_DEFINE_CATEGORY(ONSEMI)
LOG_DECLARE_CATEGORY(ONSEMI)
} // namespace libcamera

#define PADDING_SIZE 4

#define LINE_START 0x0a
#define LINE_END_TAG 0x07
#define REG_HI_BITS 0xaa
#define REG_LOW_BITS 0xa5
#define REG_VALUE 0x5a

#define SUPPORTED_BITS_PER_PIXEL 10

MdParserOnSemi::MdParserOnSemi(std::initializer_list<uint16_t> const *registerList)
{
	registerList_ = registerList;
}

MdParser::Status MdParserOnSemi::parse(libcamera::Span<const uint8_t> buffer,
				       RegisterMap &registers)
{
	if (reset_) {
		ASSERT(bitsPerPixel_ == SUPPORTED_BITS_PER_PIXEL);

		reset_ = false;
	}

	MdParser::Status ret = verifyBuffer(buffer);

	if (ret != MdParser::Status::OK)
		return ret;

	for (auto const &reg : *registerList_) {
		uint16_t regValue;
		ret = extractRegisterValue(buffer, reg, &regValue);

		if (ret != MdParser::Status::OK) {
			LOG(ONSEMI, Error) << "Failed to extract register value for 0x" << std::hex << reg << std::dec;
			reset_ = true;
			return ret;
		}

		registers[reg] = regValue;

		LOG(ONSEMI, Debug) << "Register 0x" << std::hex << reg << " = 0x" << regValue << std::dec;
	}

	return OK;
}

MdParser::Status MdParserOnSemi::verifyBuffer(libcamera::Span<const uint8_t> buffer)
{
	if ((dataWithoutPadding(buffer, 0) != LINE_START) || (dataWithoutPadding(buffer, 1) != REG_HI_BITS) || (dataWithoutPadding(buffer, 3) != REG_LOW_BITS)) {
		LOG(ONSEMI, Error) << "Invalid buffer format";
		return MdParser::Status::ERROR;
	}

	return MdParser::Status::OK;
}

uint8_t MdParserOnSemi::dataWithoutPadding(libcamera::Span<const uint8_t> buffer, unsigned int offset)
{
	offset += offset / PADDING_SIZE;

	ASSERT(offset < buffer.size());

	return buffer[offset];
}

MdParser::Status MdParserOnSemi::extractRegisterValue(libcamera::Span<const uint8_t> buffer, uint16_t registerAddress, uint16_t *registerValue)
{
	unsigned int registerOffset = 5 + (registerAddress & 0x1ff) * 2;

	MdParser::Status ret = verifyData(buffer, registerOffset);
	if (ret != MdParser::Status::OK)
		return ret;

	*registerValue = (dataWithoutPadding(buffer, registerOffset + 1) << 8) | dataWithoutPadding(buffer, registerOffset + 3);

	return MdParser::Status::OK;
}

MdParser::Status MdParserOnSemi::verifyData(libcamera::Span<const uint8_t> buffer, unsigned int offset)
{
	if ((dataWithoutPadding(buffer, offset) != REG_VALUE) ||
	    (dataWithoutPadding(buffer, offset + 2) != REG_VALUE) ||
	    (dataWithoutPadding(buffer, offset + 4) != REG_VALUE)) {
		LOG(ONSEMI, Error) << "Invalid register data format";
		return MdParser::Status::ERROR;
	}

	return MdParser::Status::OK;
}
