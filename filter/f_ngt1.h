// Copyright(c) 2018 Yohei Matsumoto, All right reserved. 

// f_ngt1.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ngt1.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ngt1.h.  If not, see <http://www.gnu.org/licenses/>. 
 
#ifndef F_NGT1_H
#define F_NGT1_H
#include "f_base.h"
#include "../channel/ch_state.h"

#include "ngt1/actisense.h"
#include "ngt1/analyzer.h"

class f_ngt1: public f_base
{
 protected:
  char m_dname[1024];
  unsigned short m_port;
  unsigned int m_br;
  AWS_SERIAL m_hserial;

  bool m_verb;
  
  //<-- these functions are from canboat.actisense-serial
  enum MSG_State
  {
    MSG_START,
    MSG_ESCAPE,
    MSG_MESSAGE
  };
  
  enum ReadyDescriptor
  {
    FD1_Ready = 0x0001,
    FD2_Ready = 0x0002
  };
  
  enum MSG_State state;
  unsigned char buf[500];
  unsigned char * head;
  RawMessage msg_raw; // I pack the msg packet into the object instead of putting into stdout as text string in original n2kMessageReceived.
  
  void readNGT1Byte(unsigned char c);
  int readNGT1(AWS_SERIAL handle);
  void messageReceived(const unsigned char * msg, size_t msgLen);
  void n2kMessageReceived(const unsigned char * msg, size_t msgLen); /* modified to pack the result into msg_raw */
  void ngtMessageReceived(const unsigned char * msg, size_t msgLen);
  // these functions are from canboat.actisense-serial -->

  // <-- these functions are from canboat.analyze
  enum GeoFormats
  {
    GEO_DD,
    GEO_DM,
    GEO_DMS
  };
  
  struct Packet
  {
    size_t lastFastPacket;
    size_t size;
    size_t allocSize;
    uint8_t * data;
  Packet():lastFastPacket(0), size(0), allocSize(0), data(NULL)
    {
    }
  };
  
  struct DevicePackets
  {
    Packet * packetList;
    DevicePackets();
    ~DevicePackets();
  };
  DevicePackets * device[256];
  const char *manufacturer[1 << 12];
  void fillManufacturers();
  void fillFieldCounts();
  bool showRaw;
  bool showData;
  bool showBytes;
  bool showJson;
  bool showJsonValue;
  bool showSI;
  
  const char * sep;
  char closingBraces[8]; // } and ] chars to close sentence in JSON mode, otherwise empty string
  enum GeoFormats showGeo;
  int onlyPgn;
  int onlySrc;
  int clockSrc;
  size_t heapSize;  
  int g_variableFieldRepeat[2]; // Actual number of repetitions
  int g_variableFieldIndex;

  bool printCanFormat(RawMessage * msg);
  void printPacket(size_t index, RawMessage * msg);
  void printCanRaw(RawMessage * msg);
  bool printPgn(RawMessage * msg, uint8_t * dataStart, int length);
  // helpers printPgn
  // from analyer.c 
  // * getSep()
  // * mprintf()
  // * printLatLon
  // * printDate
  // * printTime
  // * printPressure
  // * printTemperature
  // * print6BitASCIIText
  // * printDecimal
  // * printVarNumber
  // * printHex
  // * printNumber
  
  const char * getSep();
  bool printLatLon(char * name, double resolution, uint8_t * data, size_t bytes);
  bool printDate(char * name, uint16_t d);
  bool printTime(char * name, uint32_t t);
  bool printTemperature(char * name, uint32_t t, uint32_t bits, double resolution);
  bool printPressure(char * name, uint32_t v, Field * field);
  void print6BitASCIIChar(uint8_t b);
  bool print6BitASCIIText(char * name, uint8_t * data, size_t startBit, size_t bits);

  bool printHex(char * name, uint8_t * data, size_t startBit, size_t bits);
  bool printDecimal(char * name, uint8_t * data, size_t startBit, size_t bits);
  bool printVarNumber(char * fieldName, Pgn * pgn, uint32_t refPgn, Field * field, uint8_t * data, size_t startBit, size_t * bits);
  bool printNumber(char * fieldName, Field * field, uint8_t * data, size_t startBit, size_t bits);
  void print_ascii_json_escaped(uint8_t *data, int len);
  void setSystemClock(uint16_t currentDate, uint32_t currentTime);
  char mbuf[8192];
  char * mp;
  void mprintf(const char * format, ...);
  void mreset(void);
  void mwrite(FILE * stream);
  
  // these functions are from canboat.analyze -->
  
 public:
  f_ngt1(const char * name);
  ~f_ngt1();
  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();  
};

#endif
