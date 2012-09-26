#define BOOST_TEST_MODULE packet_test
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <libhexabus/packet.hpp>
#include "../../build/testconfig.h"


/**
 * see http://www.boost.org/doc/libs/1_43_0/libs/test/doc/html/tutorials/hello-the-testing-world.html
 */

BOOST_AUTO_TEST_CASE ( check_sanity ) {
  try {
    std::cout << "Demo test case: Checking world sanity." << std::endl;
    BOOST_CHECK_EQUAL (42, 42);
    BOOST_CHECK( 23 != 42 );        // #1 continues on error
    BOOST_REQUIRE( 23 != 42 );      // #2 throws on error

  } catch (std::exception const & ex) {
    BOOST_ERROR ( ex.what() );
  }
  if( 23 == 42 ) {
    BOOST_FAIL( "23 == 42, oh noes");             // #4 throws on error
  }
}

BOOST_AUTO_TEST_CASE ( check_write_uint8_packet_generation ) {
  std::cout << "Checking generation of uint8 write packet against stored reference packet." << std::endl;
  unsigned char testpacket[] = { 'H', 'X', '0', 'C', // Header
                        0x04,               // Packet Type: Write
                        0x00,               // Flags: None
                        0, 0, 0, 23,        // Endpoint ID: 23
                        0x02,               // Datatype: Uint8
                        42,                 // Value: 42
                        0xb1, 0x43          // CRC
                      };

  hexabus::Packet p;
  hxb_packet_int8 pi8 = p.write8(23, HXB_DTYPE_UINT8, 42, false);

  if(sizeof(testpacket) != sizeof(pi8))
    BOOST_FAIL("Size of generated packet differs from test packet");

  bool fail = false;
  for(size_t i = 0; i < sizeof(pi8); i++) {
    std::cout << "Byte " << std::dec << i << " Generated: 0x" << std::hex << (short int)((unsigned char*)&pi8)[i] << "\t" << " Reference: 0x" << (short int)testpacket[i] << std::endl;
    if(((unsigned char*)&pi8)[i] != testpacket[i])
      fail = true;
  }

  if(fail)
    BOOST_FAIL("Generated packet differs from reference packet.");
}

BOOST_AUTO_TEST_CASE ( check_write_bool_packet_generation ) {
  std::cout << "Checking generation of bool write packet against stored reference packet." << std::endl;
  unsigned char testpacket[] = { 'H', 'X', '0', 'C', // Header
                        0x04,               // Packet Type: Write
                        0x00,               // Flags: None
                        0, 0, 0, 23,        // Endpoint ID: 23
                        0x01,               // Datatype: Boolean
                        0x01,               // Value: true
                        0x04, 0xfa          // CRC
                      };

  hexabus::Packet p;
  hxb_packet_int8 pi8 = p.write8(23, HXB_DTYPE_BOOL, HXB_TRUE, false);

  if(sizeof(testpacket) != sizeof(pi8))
    BOOST_FAIL("Size of generated packet differs from test packet");

  bool fail = false;
  for(size_t i = 0; i < sizeof(pi8); i++) {
    std::cout << "Byte " << std::dec << i << " Generated: 0x" << std::hex << (short int)((unsigned char*)&pi8)[i] << "\t" << " Reference: 0x" << (short int)testpacket[i] << std::endl;
    if(((unsigned char*)&pi8)[i] != testpacket[i])
      fail = true;
  }

  if(fail)
    BOOST_FAIL("Generated packet differs from reference packet.");
}

BOOST_AUTO_TEST_CASE ( check_write_uint32_packet_generation ) {
  std::cout << "Checking generation of uint32 write packet against stored reference packet." << std::endl;
  unsigned char testpacket[] = { 'H', 'X', '0', 'C', // Header
                        0x04,               // Packet Type: Write
                        0x00,               // Flags: None
                        0, 0, 0, 42,        // Endpoint ID: 42
                        0x03,               // Datatype: Uint32
                        0xfc, 0xde, 0x41, 0xb2, // Value: 4242424242
                        0xd6, 0x3e          // CRC
                      };

  hexabus::Packet p;
  hxb_packet_int32 pi32 = p.write32(42, HXB_DTYPE_UINT32, 4242424242, false);

  if(sizeof(testpacket) != sizeof(pi32))
    BOOST_FAIL("Size of generated packet differs from test packet");

  bool fail = false;
  for(size_t i = 0; i < sizeof(pi32); i++) {
    std::cout << "Byte " << std::dec << i << " Generated: 0x" << std::hex << (short int)((unsigned char*)&pi32)[i] << "\t" << " Reference: 0x" << (short int)testpacket[i] << std::endl;
    if(((unsigned char*)&pi32)[i] != testpacket[i])
      fail = true;
  }

  if(fail)
    BOOST_FAIL("Generated packet differs from reference packet.");
}

BOOST_AUTO_TEST_CASE ( check_write_float_packet_generation ) {
  std::cout << "Checking generation of float write packet against stored reference packet." << std::endl;
  unsigned char testpacket[] = { 'H', 'X', '0', 'C', // Header
                        0x04,               // Packet Type: Write
                        0x00,               // Flags: None
                        0, 0, 0, 42,        // Endpoint ID: 42
                        0x05,               // Datatype: Uint32
                        0x41, 0xbb, 0x5c, 0x29, // Value 23.42
                        0x34, 0xf6          // CRC
                      };

  hexabus::Packet p;
  hxb_packet_float pif = p.writef(42, HXB_DTYPE_FLOAT, 23.42f, false);

  if(sizeof(testpacket) != sizeof(pif))
    BOOST_FAIL("Size of generated packet differs from test packet");

  bool fail = false;
  for(size_t i = 0; i < sizeof(pif); i++) {
    std::cout << "Byte " << std::dec << i << " Generated: 0x" << std::hex << (short int)((unsigned char*)&pif)[i] << "\t" << " Reference: 0x" << (short int)testpacket[i] << std::endl;
    if(((unsigned char*)&pif)[i] != testpacket[i])
      fail = true;
  }

  if(fail)
    BOOST_FAIL("Generated packet differs from reference packet.");
}
//BOOST_AUTO_TEST_SUITE_END()
