#pragma once

#include <vector>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <iterator>
#include <cstring>

template <typename T>
std::vector<uint8_t> Serialize(const T& object, bool little_endian = true)
{
	std::vector<uint8_t> bytes(sizeof(object));
	std::memcpy(bytes.data(), &object, sizeof(object));

	if (!little_endian)
	{
		std::reverse(bytes.begin(), bytes.end());
	}

	return bytes;
}

template <typename T>
std::vector<uint8_t> Serialize(const std::vector<T>& objects, bool little_endian = true)
{
	std::vector<uint8_t> bytes;
	for (const auto& object : objects)
	{
		std::vector<uint8_t> object_bytes = Serialize(object, little_endian);
		bytes.insert(bytes.end(), object_bytes.begin(), object_bytes.end());
	}
	return bytes;
}

template <typename T>
T DeserializeFromBytes(const std::vector<uint8_t>& bytes, bool little_endian = true)
{
	if (bytes.size() < sizeof(T))
	{
		throw std::runtime_error("Not enough bytes to convert to the target type.");
	}

	T result;
	std::vector<uint8_t> tmp(bytes.begin(), bytes.begin() + sizeof(T));

	if (!little_endian)
	{
		std::reverse(tmp.begin(), tmp.end());
	}

	std::memcpy(&result, tmp.data(), sizeof(T));

	return result;
}

template <typename T>
std::vector<T> DeserializeVectorFromBytes(const std::vector<uint8_t>& bytes, bool little_endian = true)
{
	if (bytes.size() % sizeof(T) != 0)
	{
		return {};

		//throw std::runtime_error("Byte array size is not a multiple of the target type size.");
	}

	std::vector<T> result;
	for (size_t i = 0; i < bytes.size(); i += sizeof(T))
	{
		std::vector<uint8_t> object_bytes(bytes.begin() + i, bytes.begin() + i + sizeof(T));
		result.push_back(DeserializeFromBytes<T>(object_bytes, little_endian));
	}

	return result;
}

uint8_t CalculateCRC8(const std::vector<uint8_t>& data)
{
	uint8_t crc = 0x00;
	for (auto byte : data)
	{
		crc ^= static_cast<uint8_t>(byte);
		for (int i = 0; i < 8; i++)
		{
			if ((crc & 0x80) != 0)
				crc = (uint8_t)((crc << 1) ^ 0x07);
			else
				crc <<= 1;
		}
	}
	return crc;
}

std::vector<uint8_t> COBSEncode(const std::vector<uint8_t>& data)
{
	if (data.empty())
	{
		throw std::invalid_argument("data cannot be empty.");
	}

	std::vector<uint8_t> buffer;
	buffer.reserve(data.size() + data.size() / 254 + 2);  // Extra space for overhead bytes

	auto dst = buffer.begin();
	auto code_ptr = dst;
	buffer.push_back(0);  // Initial placeholder
	++dst;
	uint8_t code = 0x01;

	for (auto ptr = data.cbegin(); ptr != data.cend(); ++ptr)
	{
		if (*ptr == 0)
		{
			*code_ptr = code;
			code_ptr = dst;
			buffer.push_back(0);  // Start a new block
			++dst;
			code = 0x01;
		}
		else
		{
			buffer.push_back(*ptr);
			++dst;
			++code;
			if (code == 0xFF)
			{
				*code_ptr = code;
				code_ptr = dst;
				buffer.push_back(0);  // Start a new block
				++dst;
				code = 0x01;
			}
		}
	}

	*code_ptr = code;

	return buffer;
}


std::optional<std::vector<uint8_t>> COBSDecode(const std::vector<uint8_t>& buffer)
{
	if (buffer.empty())
	{
		return std::nullopt;
	}

	std::vector<uint8_t> data;
	data.reserve(buffer.size());

	for (auto it = buffer.cbegin(); it != buffer.cend(); )
	{
		int code = *it++;
		for (int i = 1; i < code; i++)
		{
			if (it != buffer.cend())
			{
				data.push_back(*it++);
			}
			else
			{
				// "Invalid COBS encoding : Not enough bytes according to the code.";
				return std::nullopt;
			}
		}
		if (code < 0xFF)
		{
			if (it != buffer.cend())
			{
				data.push_back(0);
			}
			else
			{
				// We are at the end of the buffer and there's no zero byte,
				// but we'll accept it anyway, just not insert a zero into our decoded data.
				break;
			}
		}
	}

	return std::make_optional(data);
}


std::vector<uint8_t> EncodePacket(const std::vector<uint8_t>& data)
{
	std::vector<uint8_t> packet(data.begin(), data.end());

	// Calculate and add the CRC
	uint8_t crc = static_cast<uint8_t>(CalculateCRC8(packet));
	packet.push_back(crc);

	// COBS encode the packet
	packet = COBSEncode(packet);

	// Add a delimiter
	packet.push_back(0);

	return packet;
}

std::optional<std::vector<uint8_t>> DecodePacket(const std::vector<uint8_t>& packet)
{
	if (packet.empty())
	{
		return std::nullopt;
	}

	auto tDecodedPacket = COBSDecode(packet);

	if (!tDecodedPacket.has_value())
	{
		return std::nullopt;
	}

	auto decodedPacket = tDecodedPacket.value();

	// CRC check
	std::vector<uint8_t>
		data(decodedPacket.begin(), decodedPacket.end() - 1);

	uint8_t crc = static_cast<uint8_t>(CalculateCRC8(data));

	if (crc != decodedPacket.back())
	{
		// CRC failed
		return std::nullopt;
	}

	return std::make_optional(data);
}