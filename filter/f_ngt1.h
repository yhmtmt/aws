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

class PgnFieldValues;


class f_ngt1: public f_base
{
 protected:
  char m_dname[1024];
  unsigned short m_port;
  unsigned int m_br;
  AWS_SERIAL m_hserial;

  bool m_verb;

  ch_eng_state * eng_state, * eng_state2;
  
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

  static unsigned char NGT_STARTUP_SEQ[3];
  
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
    Packet();
    ~Packet();
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
  bool showTxt;
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
  bool printLatLon(char * name, double resolution, uint8_t * data, size_t bytes, PgnFieldValues * pfv);
  bool printDate(char * name, uint16_t d);
  bool printTime(char * name, uint32_t t);
  bool printTemperature(char * name, uint32_t t, uint32_t bits, double resolution, PgnFieldValues * pfv);
  bool printPressure(char * name, uint32_t v, Field * field, PgnFieldValues * pfv);
  void print6BitASCIIChar(uint8_t b);
  bool print6BitASCIIText(char * name, uint8_t * data, size_t startBit, size_t bits);

  bool printHex(char * name, uint8_t * data, size_t startBit, size_t bits, PgnFieldValues * pfv);
  bool printDecimal(char * name, uint8_t * data, size_t startBit, size_t bits, PgnFieldValues * pfv);
bool printVarNumber(char * fieldName, Pgn * pgn, uint32_t refPgn, Field * field, uint8_t * data, size_t startBit, size_t * bits, PgnFieldValues * pfv);
  bool printNumber(char * fieldName, Field * field, uint8_t * data, size_t startBit, size_t bits, PgnFieldValues * pfv);
  void print_ascii_json_escaped(uint8_t *data, int len);
  void setSystemClock(uint16_t currentDate, uint32_t currentTime);
  char mbuf[8192];
  char * mp;
  void mprintf(const char * format, ...);
  void mreset(void);
  void mwrite(FILE * stream);
  
  void writeMessage(AWS_SERIAL handle, unsigned char command, const unsigned char * cmd,
		    const size_t len);

  // these functions are from canboat.analyze -->

  list<PgnFieldValues*> pgn_queue;

  void handle_pgn_eng_state(PgnFieldValues * pfv,
			    ch_eng_state * ch, const unsigned char ieng = 0);
  
  ////////////////////////////////////////// pgn hanler
  
 public:
  f_ngt1(const char * name);
  ~f_ngt1();
  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();
};


class FieldValueBase
{
 private:
 public:
  virtual void print() const = 0;
  virtual void print_type() const = 0;
};

template <class T> class FieldValue: public FieldValueBase
{
 private:
  T val;
  
 public:
 FieldValue(const T & _val):val(_val)
  {
  }

  const T & get(){
    return val;
  }

  void set(const T & _val){
    val = _val;
  }

  virtual void print() const 
  {
    cout << val;
  }

  virtual void print_type() const
  {
    cout << typeid(T).name() << "(" << sizeof(T) << ")";
  }
};

class PgnFieldValues
{
 private:
  const Pgn * pgn;
  vector<FieldValueBase*> values;
 public:
  PgnFieldValues(const Pgn * _pgn);
  ~PgnFieldValues();

  const uint32_t getPgn(){
    if(pgn)
      return pgn->pgn;
    else
      return 0;
  }
  
  const int getNumFields()
  {
    return (int) values.size();
  }
  
  const FieldValueBase * get(const int ifield)
  {
    if(ifield < 0 || ifield >= (int)values.size())
      return NULL;
    return values[ifield];
  }

  template<class T> const T * get_vptr(const int ifield){
    if(ifield < 0 || ifield >= (int)values.size())
      return NULL;

    FieldValue<T> * pfv = dynamic_cast<FieldValue<T>*>(values[ifield]);
    if(pfv)
      return &(pfv->get());
    cout << "field[" << ifield << "] is null, the type is ";
    values[ifield]->print_type();
    cout << " val=";
    values[ifield]->print();
    cout << endl;
    return NULL;
  }
  
  template<class T> void set(const int ifield, const T & val)
  {

    if(ifield < 0 || ifield >= values.size())
      return;
    FieldValue<T> * pfv = dynamic_cast<FieldValue<T>*>(values[ifield]);
    
    pfv->set(val);
  }
  
  template <class T > void push(const T & val)
    {
      FieldValue<T> * p = new FieldValue<T>(val);    
      if(!p){
	cerr << "Failed to allocate memory" << endl;
	exit(1);
      }
      
      values.push_back(p);
    }  
};

#endif
