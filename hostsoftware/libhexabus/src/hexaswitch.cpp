#include <iostream>
#include <string.h>
#include <iomanip>
#include <libhexabus/common.hpp>
#include <libhexabus/crc.hpp>
#include <libhexabus/liveness.hpp>
#include <libhexabus/socket.hpp>
#include <libhexabus/packet.hpp>
#include <libhexabus/error.hpp>
#include <time.h>
#include <boost/program_options.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <typeinfo>
namespace po = boost::program_options;

#include "../../../shared/endpoints.h"
#include "resolv.hpp"

#pragma GCC diagnostic warning "-Wstrict-aliasing"

enum ErrorCode {
	ERR_NONE = 0,

	ERR_UNKNOWN_PARAMETER = 1,
	ERR_PARAMETER_MISSING = 2,
	ERR_PARAMETER_FORMAT = 3,
	ERR_PARAMETER_VALUE_INVALID = 4,
	ERR_NETWORK = 5,

	ERR_OTHER = 127
};

struct PacketPrinter : public hexabus::PacketVisitor {
	private:
		std::ostream& target;

		void printValueHeader(uint32_t eid, const char* datatypeStr)
		{
			target << "Info" << std::endl
				<< "Endpoint ID:\t" << eid << std::endl
				<< "Datatype:\t" << datatypeStr << std::endl;
		}

		template<typename T>
		void printValuePacket(const hexabus::ValuePacket<T>& packet, const char* datatypeStr)
		{
			printValueHeader(packet.eid(), datatypeStr);
			target << "Value:\t" << packet.value() << std::endl;
			target << std::endl;
		}

		void printValuePacket(const hexabus::ValuePacket<uint8_t>& packet, const char* datatypeStr)
		{
			printValueHeader(packet.eid(), datatypeStr);
			target << "Value:\t" << (int) packet.value() << std::endl;
			target << std::endl;
		}

		template<size_t L>
		void printValuePacket(const hexabus::ValuePacket<boost::array<char, L> >& packet, const char* datatypeStr)
		{
			printValueHeader(packet.eid(), datatypeStr);

			std::stringstream hexstream;

			hexstream << std::hex << std::setfill('0');

			for (size_t i = 0; i < L; ++i) {
				hexstream << std::setw(2) << (0xFF & packet.value()[i]) << " ";
			}

			std::cout << std::endl << std::endl;

			target << "Value:\t" << hexstream.str() << std::endl; 
			target << std::endl;
		}

	public:
		PacketPrinter(std::ostream& target)
			: target(target)
		{}

		virtual void visit(const hexabus::ErrorPacket& error)
		{
			target << "Error" << std::endl
				<< "Error code:\t";
			switch (error.code()) {
				case HXB_ERR_UNKNOWNEID:
					target << "Unknown EID";
					break;
				case HXB_ERR_WRITEREADONLY:
					target << "Write on readonly endpoint";
					break;
				case HXB_ERR_CRCFAILED:
					target << "CRC failed";
					break;
				case HXB_ERR_DATATYPE:
					target << "Datatype mismatch";
					break;
				case HXB_ERR_INVALID_VALUE:
					target << "Invalid value";
					break;
				default:
					target << "(unknown)";
					break;
			}
			target << std::endl;
			target << std::endl;
		}

		virtual void visit(const hexabus::QueryPacket& query) {}
		virtual void visit(const hexabus::EndpointQueryPacket& endpointQuery) {}

		virtual void visit(const hexabus::EndpointInfoPacket& endpointInfo)
		{
			if (endpointInfo.eid() == 0) {
				target << "Device Info" << std::endl
					<< "Device Name:\t" << endpointInfo.value() << std::endl;
			} else {
				target << "Endpoint Info\n"
					<< "Endpoint ID:\t" << endpointInfo.eid() << std::endl
					<< "EP Datatype:\t";
				switch(endpointInfo.datatype()) {
					case HXB_DTYPE_BOOL:
						target << "Bool";
						break;
					case HXB_DTYPE_UINT8:
						target << "UInt8";
						break;
					case HXB_DTYPE_UINT32:
						target << "UInt32";
						break;
					case HXB_DTYPE_DATETIME:
						target << "Datetime";
						break;
					case HXB_DTYPE_FLOAT:
						target << "Float";
						break;
					case HXB_DTYPE_TIMESTAMP:
						target << "Timestamp";
						break;
					case HXB_DTYPE_128STRING:
						target << "String";
						break;
					case HXB_DTYPE_16BYTES:
						target << "Binary (16bytes)";
						break;
					case HXB_DTYPE_66BYTES:
						target << "Binary (66bytes)";
						break;
					default:
						target << "(unknown)";
						break;
				}
				target << std::endl;
				target << "EP Name:\t" << endpointInfo.value() << std::endl;
			}
			target << std::endl;
		}

		virtual void visit(const hexabus::InfoPacket<bool>& info) { printValuePacket(info, "Bool"); }
		virtual void visit(const hexabus::InfoPacket<uint8_t>& info) { printValuePacket(info, "UInt8"); }
		virtual void visit(const hexabus::InfoPacket<uint32_t>& info) { printValuePacket(info, "UInt32"); }
		virtual void visit(const hexabus::InfoPacket<float>& info) { printValuePacket(info, "Float"); }
		virtual void visit(const hexabus::InfoPacket<boost::posix_time::ptime>& info) { printValuePacket(info, "Datetime"); }
		virtual void visit(const hexabus::InfoPacket<boost::posix_time::time_duration>& info) { printValuePacket(info, "Timestamp"); }
		virtual void visit(const hexabus::InfoPacket<std::string>& info) { printValuePacket(info, "String"); }
		virtual void visit(const hexabus::InfoPacket<boost::array<char, HXB_16BYTES_PACKET_MAX_BUFFER_LENGTH> >& info) { printValuePacket(info, "Binary (16 bytes)"); }
		virtual void visit(const hexabus::InfoPacket<boost::array<char, HXB_66BYTES_PACKET_MAX_BUFFER_LENGTH> >& info) { printValuePacket(info, "Binary (66 bytes)"); }

		virtual void visit(const hexabus::WritePacket<bool>& write) {}
		virtual void visit(const hexabus::WritePacket<uint8_t>& write) {}
		virtual void visit(const hexabus::WritePacket<uint32_t>& write) {}
		virtual void visit(const hexabus::WritePacket<float>& write) {}
		virtual void visit(const hexabus::WritePacket<boost::posix_time::ptime>& write) {}
		virtual void visit(const hexabus::WritePacket<boost::posix_time::time_duration>& write) {}
		virtual void visit(const hexabus::WritePacket<std::string>& write) {}
		virtual void visit(const hexabus::WritePacket<boost::array<char, HXB_16BYTES_PACKET_MAX_BUFFER_LENGTH> >& write) {}
		virtual void visit(const hexabus::WritePacket<boost::array<char, HXB_66BYTES_PACKET_MAX_BUFFER_LENGTH> >& write) {}
};

void print_packet(const hexabus::Packet& packet)
{
	PacketPrinter pp(std::cout);

	pp.visitPacket(packet);
}

ErrorCode send_packet(hexabus::Socket* net, const boost::asio::ip::address_v6& addr, const hexabus::Packet& packet)
{
  try {
    net->send(packet, addr);
  } catch (const hexabus::NetworkException& e) {
    std::cerr << "Could not send packet to " << addr << ": " << e.code().message() << std::endl;
		return ERR_NETWORK;
  }

	return ERR_NONE;
}

template<typename Filter>
ErrorCode send_packet_wait(hexabus::Socket* net, const boost::asio::ip::address_v6& addr, const hexabus::Packet& packet, const Filter& filter)
{
	ErrorCode err;
	
	if ((err = send_packet(net, addr, packet))) {
		return err;
	}

	try {
		std::pair<hexabus::Packet::Ptr, boost::asio::ip::udp::endpoint> p = net->receive(filter && hexabus::filtering::sourceIP() == addr);
		
		print_packet(*p.first);
	} catch (const hexabus::NetworkException& e) {
		std::cerr << "Error receiving packet: " << e.code().message() << std::endl;
		return ERR_NETWORK;
	} catch (const hexabus::GenericException& e) {
		std::cerr << "Error receiving packet: " << e.what() << std::endl;
		return ERR_NETWORK;
	}
	
	return ERR_NONE;
}

template<template<typename TValue> class ValuePacket>
ErrorCode send_value_packet(hexabus::Socket* net, const boost::asio::ip::address_v6& ip, uint32_t eid, uint8_t datatype, const std::string& value)
{
	try { // handle errors in value lexical_cast
		switch (datatype) {
			case HXB_DTYPE_BOOL:
				{
					bool b = boost::lexical_cast<unsigned int>(value);
					std::cout << "Sending value " << b << std::endl;
					return send_packet(net, ip, ValuePacket<bool>(eid, b));
				}

			case HXB_DTYPE_UINT8:
				{
					uint8_t u8 = boost::lexical_cast<unsigned int>(value);
					std::cout << "Sending value " << (unsigned int) u8 << std::endl;
					return send_packet(net, ip, ValuePacket<uint8_t>(eid, u8));
				}

			case HXB_DTYPE_UINT32:
				{
					uint32_t u32 = boost::lexical_cast<uint32_t>(value);
					std::cout << "Sending value " << u32 << std::endl;
					return send_packet(net, ip, ValuePacket<uint32_t>(eid, u32));
				}

			case HXB_DTYPE_FLOAT:
				{
					float f = boost::lexical_cast<float>(value);
					std::cout << "Sending value " << f << std::endl;
					return send_packet(net, ip, ValuePacket<float>(eid, f));
				}

			case HXB_DTYPE_128STRING:
				{
					std::cout << "Sending value " << value << std::endl;
					return send_packet(net, ip, ValuePacket<std::string>(eid, value));
				}

			default:
				std::cout << "Unknown data type " << datatype << std::endl;
				return ERR_PARAMETER_VALUE_INVALID;
		}
	} catch (boost::bad_lexical_cast& e) {
		std::cerr << "Error while converting value: " << e.what() << std::endl;
		return ERR_PARAMETER_VALUE_INVALID;
	}
}

enum Command {
	C_GET,
	C_SET,
	C_EPQUERY,
	C_SEND,
	C_LISTEN,
	C_ON,
	C_OFF,
	C_STATUS,
	C_POWER,
	C_DEVINFO
};

int main(int argc, char** argv) {

  std::ostringstream oss;
  oss << "Usage: " << argv[0] << " ACTION IP [additional options] ";
  po::options_description desc(oss.str());
  desc.add_options()
    ("help,h", "produce help message")
    ("version", "print libhexabus version and exit")
    ("command,c", po::value<std::string>(), "{get|set|epquery|send|listen|on|off|status|power|devinfo}")
    ("ip,i", po::value<std::string>(), "the hostname to connect to")
    ("bind,b", po::value<std::string>(), "local IP address to use")
    ("interface,I", po::value<std::string>(), "the interface to use for outgoing messages")
    ("eid,e", po::value<uint32_t>(), "Endpoint ID (EID)")
    ("datatype,d", po::value<unsigned int>(), "{1: Bool | 2: UInt8 | 3: UInt32 | 4: HexaTime | 5: Float | 6: String}")
    ("value,v", po::value<std::string>(), "Value")
    ("reliable,r", po::bool_switch(), "Reliable initialization of network access (adds delay, only needed for broadcasts)")
    ;
  po::positional_options_description p;
  p.add("command", 1);
  p.add("ip", 1);
  po::variables_map vm;

  // Begin processing of commandline parameters.
  try {
    po::store(po::command_line_parser(argc, argv).
        options(desc).positional(p).run(), vm);
    po::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Cannot process commandline options: " << e.what() << std::endl;
		return ERR_UNKNOWN_PARAMETER;
  }

	Command command;
	boost::optional<boost::asio::ip::address_v6> ip;
	boost::optional<std::string> interface;
  boost::asio::ip::address_v6 bind_addr(boost::asio::ip::address_v6::any());

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 0;
  }

  if (vm.count("version")) {
    std::cout << "hexaswitch -- command line hexabus client" << std::endl;
    std::cout << "libhexabus version " << hexabus::version() << std::endl;
    return 0;
  }

  if (! vm.count("command")) {
    std::cerr << "You must specify a command." << std::endl;
    return ERR_PARAMETER_MISSING;
  } else {
		std::string command_str = vm["command"].as<std::string>();

		if (boost::iequals(command_str, "GET")) {
			command = C_GET;
		} else if (boost::iequals(command_str, "SET")) {
			command = C_SET;
		} else if (boost::iequals(command_str, "EPQUERY")) {
			command = C_EPQUERY;
		} else if (boost::iequals(command_str, "SEND")) {
			command = C_SEND;
		} else if (boost::iequals(command_str, "LISTEN")) {
			command = C_LISTEN;
		} else if (boost::iequals(command_str, "ON")) {
			command = C_ON;
		} else if (boost::iequals(command_str, "OFF")) {
			command = C_OFF;
		} else if (boost::iequals(command_str, "STATUS")) {
			command = C_STATUS;
		} else if (boost::iequals(command_str, "POWER")) {
			command = C_POWER;
		} else if (boost::iequals(command_str, "DEVINFO")) {
			command = C_DEVINFO;
		} else {
			std::cerr << "Unknown command \"" << command_str << "\"" << std::endl;
			return ERR_PARAMETER_VALUE_INVALID;
		}
	}

	boost::asio::io_service io;

	if (vm.count("ip")) {
		boost::system::error_code err;
		ip = hexabus::resolve(io, vm["ip"].as<std::string>(), err);
		if (err) {
			std::cerr << vm["ip"].as<std::string>() << " is not a valid IP address: " << err.message() << std::endl;
			return ERR_PARAMETER_VALUE_INVALID;
		}
	}

  if (vm.count("bind")) {
		boost::system::error_code err;
    bind_addr = hexabus::resolve(io, vm["bind"].as<std::string>(), err);
		if (err) {
			std::cerr << vm["bind"].as<std::string>() << " is not a valid IP address: " << err.message() << std::endl;
			return ERR_PARAMETER_VALUE_INVALID;
		}
    std::cout << "Binding to " << bind_addr << std::endl;
  }

	if (vm.count("interface")) {
		interface = vm["interface"].as<std::string>();
	}

	if (command == C_LISTEN && !interface) {
		std::cerr << "Command LISTEN requires an interface to listen on." << std::endl;
		return ERR_PARAMETER_MISSING;
	}

  hexabus::Socket* network;

  if (interface) {
    std::cout << "Using interface " << *interface << std::endl;
    try {
      network = new hexabus::Socket(io, *interface);
    } catch (const hexabus::NetworkException& e) {
      std::cerr << "Could not open socket on interface " << *interface << ": " << e.code().message() << std::endl;
			return ERR_NETWORK;
    }
  } else {
    try {
      network = new hexabus::Socket(io);
    } catch (const hexabus::NetworkException& e) {
      std::cerr << "Could not open socket: " << e.code().message() << std::endl;
			return ERR_NETWORK;
    }
  }

	if (command == C_LISTEN) {
    std::cout << "Entering listen mode." << std::endl;
		try {
			network->listen(bind_addr);
		} catch (const hexabus::NetworkException& e) {
			std::cerr << "Cannot listen on " << bind_addr << ": " << e.code().message() << std::endl;
			return ERR_NETWORK;
		}

		while (true) {
			std::pair<hexabus::Packet::Ptr, boost::asio::ip::udp::endpoint> pair;
			try {
				pair = network->receive();
			} catch (const hexabus::NetworkException& e) {
				std::cerr << "Error receiving packet: " << e.code().message() << std::endl;
				return ERR_NETWORK;
			} catch (const hexabus::GenericException& e) {
				std::cerr << "Error receiving packet: " << e.what() << std::endl;
				return ERR_NETWORK;
			}

			std::cout << "Received packet from " << pair.second << std::endl;
			print_packet(*pair.first);
		}
	}

	if (!ip && command == C_SEND) {
		ip = hexabus::Socket::GroupAddress;
	} else if (!ip) {
		std::cerr << "Command requires an IP address" << std::endl;
		return ERR_PARAMETER_MISSING;
	}
	if (ip->is_multicast() && !interface) {
		std::cerr << "Sending to multicast requires an interface to send from" << std::endl;
		return ERR_PARAMETER_MISSING;
	}

	try {
		network->bind(bind_addr);
	} catch (const hexabus::NetworkException& e) {
		std::cerr << "Cannot bind to " << bind_addr << ": " << e.code().message() << std::endl;
		return ERR_NETWORK;
	}

	hexabus::LivenessReporter* liveness =
    vm.count("reliable") && vm["reliable"].as<bool>()
    ? new hexabus::LivenessReporter(*network)
    : 0;
	if (liveness) {
		liveness->establishPaths(1);
		liveness->start();
	}

	switch (command) {
		case C_ON:
		case C_OFF:
			return send_packet(network, *ip, hexabus::WritePacket<bool>(EP_POWER_SWITCH, command == C_ON));

		case C_STATUS:
			return send_packet(network, *ip, hexabus::QueryPacket(EP_POWER_SWITCH));

		case C_POWER:
			return send_packet(network, *ip, hexabus::QueryPacket(EP_POWER_METER));

		case C_SET:
		case C_SEND:
			{
				if (!vm.count("eid")) {
					std::cerr << "Command needs an EID" << std::endl;
					return ERR_PARAMETER_MISSING;
				}
				if (!vm.count("datatype")) {
					std::cerr << "Command needs a data type" << std::endl;
					return ERR_PARAMETER_MISSING;
				}
				if (!vm.count("value")) {
					std::cerr << "Command needs a value" << std::endl;
					return ERR_PARAMETER_MISSING;
				}

				try {
					uint32_t eid = vm["eid"].as<uint32_t>();
					unsigned int dtype = vm["datatype"].as<unsigned int>();
					std::string value = vm["value"].as<std::string>();

					if (command == C_SET) {
						return send_value_packet<hexabus::WritePacket>(network, *ip, eid, dtype, value);
					} else {
						return send_value_packet<hexabus::InfoPacket>(network, *ip, eid, dtype, value);
					}
				} catch (const std::exception& e) {
					std::cerr << "Cannot process option: " << e.what() << std::endl;
					return ERR_UNKNOWN_PARAMETER;
				}
			}

		case C_GET:
		case C_EPQUERY:
			{
				if (!vm.count("eid")) {
					std::cerr << "Command needs an EID" << std::endl;
					return ERR_PARAMETER_MISSING;
				}
				uint32_t eid = vm["eid"].as<uint32_t>();

				if (command == C_GET) {
					return send_packet_wait(network, *ip, hexabus::QueryPacket(eid), hexabus::filtering::eid() == eid);
				} else {
					return send_packet_wait(network, *ip, hexabus::EndpointQueryPacket(eid), hexabus::filtering::eid() == eid);
				}
			}

		case C_DEVINFO:
			return send_packet_wait(network, *ip, hexabus::EndpointQueryPacket(EP_DEVICE_DESCRIPTOR), hexabus::filtering::eid() == EP_DEVICE_DESCRIPTOR);

		default:
			std::cerr << "BUG: Unknown command" << std::endl;
			return ERR_OTHER;
	}

	return ERR_NONE;
}

