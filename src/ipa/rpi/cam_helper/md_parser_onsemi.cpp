#include <libcamera/base/log.h>

#include "md_parser.h"

using namespace RPiController;
using namespace libcamera;

namespace libcamera {
LOG_DEFINE_CATEGORY(ONSEMI)
LOG_DECLARE_CATEGORY(ONSEMI)
} // namespace libcamera

#define TAG_LINE_START 0x0A
#define TAG_ADDR_MSB 0xAA
#define TAG_ADDR_LSB 0xA5
#define TAG_DATA 0x5A
#define TAG_END_OF_DATA 0x07

#define REG_SIZE_BYTES 2 // Data bytes
#define REG_PACKET_SIZE_BYTES 4 // Tag bytes + data bytes

MdParserOnSemi::MdParserOnSemi(std::initializer_list<uint16_t> const *registerList)
{
	for (auto reg_address : *registerList)
		offsets_[reg_address] = {};
}

MdParser::Status MdParserOnSemi::parse(libcamera::Span<const uint8_t> buffer,
				       RegisterMap &registers)
{
	MdParser::Status ret;

	if (reset_) {
		ASSERT((bitsPerPixel_ == 8) || (bitsPerPixel_ == 10) || (bitsPerPixel_ == 12));

		if (bitsPerPixel_ > 8)
			paddingInterval_ = (uint8_t)(8 / (bitsPerPixel_ - 8));

		ret = findRegs(buffer);
		if (ret != MdParser::Status::OK)
			return ret;

		reset_ = false;
	}

	registers.clear();
	for (const auto &[reg_address, offset] : offsets_) {
		if (!offset) {
			reset_ = true;
			return NOTFOUND;
		}

		uint16_t registerValue;
		ret = getValue(buffer, offset.value(), &registerValue, TAG_DATA, TAG_DATA);

		if (ret != OK) {
			reset_ = true;
			return ret;
		}

		registers[reg_address] = registerValue;
	}

	return OK;
}

MdParser::Status MdParserOnSemi::findRegs(libcamera::Span<const uint8_t> buffer)
{
	MdParser::Status ret;
	bool wait_for_line_start = true, parse = true;
	uint8_t embedded_line = 0;
	uint16_t index = 0, current_reg_addr = 0;
	uint16_t index_amount = buffer.size();
	OffsetMap::iterator it = offsets_.begin();

	if (paddingInterval_ > 0)
		index_amount -= buffer.size() / (paddingInterval_ + 1);

	while ((index < index_amount) && (it != offsets_.end()) && parse) {
		uint8_t tag = dataWithoutPadding(buffer, index);

		switch (tag) {
		case TAG_DATA:
			if (it->first == current_reg_addr) {
				LOG(ONSEMI, Debug) << "Register 0x" << std::hex << it->first << " found at " << std::dec << index;
				it->second = index;
				it++;
			} else if (it->first < current_reg_addr) {
				LOG(ONSEMI, Error) << "Register 0x" << std::hex << it->first << " not found, abort..." << std::dec;
				parse = false;
			}

			current_reg_addr += REG_SIZE_BYTES;
			index += REG_PACKET_SIZE_BYTES;
			break;
		case TAG_ADDR_MSB:
			ret = getValue(buffer, index, &current_reg_addr, TAG_ADDR_MSB, TAG_ADDR_LSB);
			if (ret != OK)
				return ret;

			index += REG_PACKET_SIZE_BYTES;
			break;
		case TAG_LINE_START:
			embedded_line++;
			wait_for_line_start = false;
			LOG(ONSEMI, Debug) << "Line " << (int)embedded_line << " start at " << index;
			index++;
			break;
		case TAG_END_OF_DATA:
			if (!wait_for_line_start) {
				LOG(ONSEMI, Debug) << "End of line " << (int)embedded_line << " at " << index;

				if (embedded_line == numLines_) {
					LOG(ONSEMI, Debug) << "All embedded lines parsed. Finish";
					parse = false;
				}
			}

			wait_for_line_start = true;
			index++;
			break;
		default:
			LOG(ONSEMI, Error) << "Unexpected tag 0x" << std::hex << (unsigned int)tag << " at " << std::dec << index;
			return ERROR;
		}
	}

	if (it != offsets_.end()) {
		LOG(ONSEMI, Error) << "Couldn't find reg 0x" << std::hex << it->first << std::dec;
		return NOTFOUND;
	}

	return OK;
}

MdParser::Status MdParserOnSemi::getValue(libcamera::Span<const uint8_t> buffer,
					  uint16_t index, uint16_t *value, uint8_t tag0, uint8_t tag1)
{
	if ((dataWithoutPadding(buffer, index) != tag0) || (dataWithoutPadding(buffer, index + REG_SIZE_BYTES) != tag1)) {
		LOG(ONSEMI, Error) << "Incorrect register value tags at " << index;
		return ERROR;
	}

	index++;
	*value = (dataWithoutPadding(buffer, index) << 8) | dataWithoutPadding(buffer, index + REG_SIZE_BYTES);

	return OK;
}

uint8_t MdParserOnSemi::dataWithoutPadding(libcamera::Span<const uint8_t> buffer,
					   uint16_t offset)
{
	if (paddingInterval_ > 0)
		offset += offset / paddingInterval_;

	ASSERT(offset < buffer.size());

	return buffer[offset];
}
