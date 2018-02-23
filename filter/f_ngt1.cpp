// Copyright(c) 2018 Yohei Matsumoto, All right reserved. 

// f_ngt1.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ngt1.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ngt1.cpp. If not, see <http://www.gnu.org/licenses/>. 
 
#include "stdafx.h"

#include <cmath>
#include <cstring>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>

using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/aws_serial.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/timeb.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <ctype.h>
#include <stdarg.h>
#include <stdint.h>


#include "ngt1/common.h"
#include "f_ngt1.h"



PgnFieldValues::PgnFieldValues(const Pgn * _pgn):pgn(_pgn)
{
}

PgnFieldValues::~PgnFieldValues()
{
  for(int ifield = 0; ifield < values.size(); ifield++)
    delete values[ifield];
  values.clear();
}

////////////////////////////////////////////////////////////////////


f_ngt1::Packet::Packet():lastFastPacket(0), size(0), allocSize(0), data(NULL)
{
}

f_ngt1::Packet::~Packet()
{
  if(data){
    free(data);
    data = NULL;
  }
}


f_ngt1::DevicePackets::DevicePackets():packetList(NULL)
{
  packetList = new Packet[pgnListSize];
}

f_ngt1::DevicePackets::~DevicePackets()
{
  if(packetList){
    delete[] packetList;
    packetList = NULL;
  }
}

unsigned char f_ngt1::NGT_STARTUP_SEQ[3] =
  { 
    0x11   /* msg byte 1, meaning ? */
    , 0x02   /* msg byte 2, meaning ? */
    , 0x00   /* msg byte 3, meaning ? */
  };


f_ngt1::f_ngt1(const char * name):f_base(name), eng_state(NULL), eng_state2(NULL),  m_hserial(NULL_SERIAL), state(MSG_START), showRaw(false), showTxt(false), showData(false), showBytes(false), showJson(false), showSI(false), sep(NULL), onlyPgn(0), onlySrc(-1), clockSrc(-1), heapSize(0), showGeo(GEO_DD), mp(mbuf)
{
  register_fpar("ch_eng_state", (ch_base**)&eng_state, typeid(ch_eng_state).name(), "Channel for engine state");
  register_fpar("ch_eng_state2", (ch_base**)&eng_state2, typeid(ch_eng_state).name(), "Channel for second engine state");
  register_fpar("dev", m_dname, 1024, "Device file path of the serial port.");
  register_fpar("port", &m_port, "Port number of the serial port. (only for windows)");

  register_fpar("br", &m_br, "Baudrate.");
  register_fpar("verb", &m_verb, "Verbose mode.");
  register_fpar("ShowTxt", &showTxt, "Text output is prepared (plain or Json).");
  register_fpar("ShowJson", &showJson, "Text formated as Json format.");
}

f_ngt1::~f_ngt1()
{
}

bool f_ngt1::init_run()
{
#ifdef _WIN32
  m_hserial = open_serial(m_port, m_br);
#else
  m_hserial = open_serial(m_dname, m_br);
#endif
  if (m_hserial == NULL_SERIAL)
    return false;
  
  writeMessage(m_hserial, NGT_MSG_SEND, NGT_STARTUP_SEQ, sizeof(NGT_STARTUP_SEQ));

  memset((void*)device, 0, sizeof(device));
  fillManufacturers();
  fillFieldCounts();
  checkPgnList();
 
  head = buf;
  heapSize = 0;
  mp = mbuf;
  
  return true;
}


void f_ngt1::destroy_run()
{
  if(m_hserial != NULL_SERIAL)
    close_serial(m_hserial);

  // release device-packet data
  for(int idev = 0; idev < 256; idev++){
    if(device[idev]){
      delete device[idev];
      device[idev] = NULL;
    }
  }

  for(auto itr = pgn_queue.begin(); itr != pgn_queue.end(); itr++)
    delete *itr;

  pgn_queue.clear();
  heapSize = 0;  
}

bool f_ngt1::proc()
{
  readNGT1(m_hserial);

  auto itr = pgn_queue.begin();
  while(pgn_queue.size()){
    PgnFieldValues * pfv = *itr;
    
    if(pfv){
     
      handle_pgn_eng_state(pfv, eng_state, 0);
      handle_pgn_eng_state(pfv, eng_state2, 1);

      /*
      if(m_verb){
	cout << "PGN " << pfv->getPgn() << endl;
	for(int ifv = 0; ifv < pfv->getNumFields(); ifv++){
	  const FieldValueBase * pfvb = pfv->get(ifv);
	  cout << "Field[" << ifv << "]:";
	  pfvb->print();
	  cout << " type=";
	  pfvb->print_type();
	  cout << endl;
	}
      }
      */
      delete *itr;
    }
    itr = pgn_queue.erase(itr);
  }
  return true;
}

void f_ngt1::handle_pgn_eng_state(PgnFieldValues * pfv, ch_eng_state * ch, const unsigned char ieng)
{
  if(ch){
    // 127488 engine parameters(rapid)
    // 0. Engine Instance, 1. Engine Speed(rpm), 2. Engine Boost Pressure(hPa),
    // 3. Engine Trim(?)
    //
    // 127489 engine parameters(dynamic)
    // 0. Engine Instance, 1. Oil Pressure(hPa), 2. Oil temperature(K),
    // 3. Temperature(K), 4. Alt Potential(V), 5. Fuel Rate(L/h),
    // 6. Total Engine hours(s), 7. Coolant Pressure(hPa),
    // 8. Fuel Pressure(hPa), 9. Reserved, 10 Status1,
    // 11. Status2, 12. Engine Load(%), 13. Engine Torque(%) 
    //
    // 127493 Transmission Parameters (Dynamic)
    // 0. Engine Instance, 1. Transmission Gear, 2. Reserved,
    // 3. Oil pressure(hPa), 4. Oil temperature, 5. Status 1
    //
    // 127496 Trip Parameters(Vessel)
    // 0. Time to Empty(s), 1. Distance to Empty(m), 2. Fuel Remaining(L)
    // 3. Trip Run Time(s)
    //
    // 127497 Trip Parameters(Engine)
    // 0. Engine Instance, 1. Fuel Rate avg(L/h), 2. Fuel Rate eco (L/h)
    // 3. Instant Fuel eco (L/h)
    // 
    // 127498 Engine Parameters(Static)
    // 0. Engine Instance, 1. Rated Engine Speed, 2. Vin, 3. Software ID

    switch(pfv->getPgn()){
    case 127488: // rapid engine parameter
      if(*pfv->get_vptr<int64_t>(0) != ieng)
	break;
      ch->set_rapid(get_time(),
		    (float)((double)*pfv->get_vptr<int64_t>(1) * 0.25),
		    (char)(*pfv->get_vptr<int64_t>(3)));
      if(m_verb)
	cout << 127488 << " rpm:" << (float)((double)*pfv->get_vptr<int64_t>(1) * 0.25) << " trim:" << (*pfv->get_vptr<int64_t>(3)) << endl;
      break;
    case 127489:
      if(*pfv->get_vptr<int64_t>(0) != ieng)
	break;
      {
	int32_t poil=(int)(*pfv->get_vptr<int32_t>(1));
	float toil = (float)(*pfv->get_vptr<double>(2));
	float temp = (float)(*pfv->get_vptr<double>(3));
	float valt = (float)((double)*pfv->get_vptr<int64_t>(4) * 0.01);
	float frate = (float)((double)*pfv->get_vptr<int64_t>(5) * 0.1);
	unsigned int teng = (unsigned int)(*pfv->get_vptr<int64_t>(6));
	int pclnt = (int)(*pfv->get_vptr<int32_t>(7));
	int pfl = (int)(*pfv->get_vptr<int64_t>(8));
	StatEng1 stat1 = (StatEng1)(*pfv->get_vptr<int64_t>(9));
	StatEng2 stat2 = (StatEng2)(*pfv->get_vptr<int64_t>(10));
	unsigned char ld = (unsigned char)(*pfv->get_vptr<int64_t>(11));
	unsigned char tq = (unsigned char)(*pfv->get_vptr<int64_t>(12));
	ch->set_dynamic(get_time(), poil, toil, temp, valt, frate, teng, pclnt, pfl, stat1, stat2, ld, tq);
	if(m_verb)
	  cout << 127489 << " temp:" << temp << " valt:" << valt << " poil:" << poil << endl; 
      }
      break;
    case 127493:
      if(*pfv->get_vptr<int64_t>(0) != ieng)
	break;
      ch->set_tran(get_time(),
		   (StatGear)(*pfv->get_vptr<int64_t>(1)),
		   (int)(*pfv->get_vptr<int64_t>(3)),
		   (float)(*pfv->get_vptr<double>(4)));
      if(m_verb)
	cout << 127493 << " Gear:" << (*pfv->get_vptr<int64_t>(1)) << endl;
      break;
    case 127497:
      if(*pfv->get_vptr<int64_t>(0) != ieng)
	break;
      ch->set_trip(get_time(),
		   (int)(*pfv->get_vptr<int64_t>(1)),
		   (float)((double)*pfv->get_vptr<int64_t>(2) * 0.1),
		   (float)((double)*pfv->get_vptr<int64_t>(3) * 0.1),
		   (float)((double)*pfv->get_vptr<int64_t>(4) * 0.1));			      if(m_verb)									  cout << 127497 << " fuel used:" << (int)(*pfv->get_vptr<int64_t>(1)) << " Fuel Rate:" << (float)((double)*pfv->get_vptr<int64_t>(2) * 0.1) << endl;
      break;
    }
  }
}

///////////////////////////////////////////////////////////// from canboat
int f_ngt1::readNGT1(AWS_SERIAL handle)
{
  int r;
  unsigned char c;
  unsigned char buf_tmp[500];

  r = read_serial(handle, (char*)buf_tmp, sizeof(buf_tmp));
  //  r = read(handle, buf, sizeof(buf));

  for (int i = 0; i < r; i++)
  {
    c = buf_tmp[i];
    readNGT1Byte(c);
  }
  return r;
}


void  f_ngt1::readNGT1Byte(unsigned char c)
{
  if (state == MSG_ESCAPE)
  {
    if (c == ETX)
    {
      messageReceived(buf, head - buf);
      head = buf;
      state = MSG_START;
    }
    else if (c == STX)
    {
      head = buf;
      state = MSG_MESSAGE;
    }
    else if (c == DLE)
    {
      *head++ = c;
      state = MSG_MESSAGE;
    }
    else
    {
      printf("DLE followed by unexpected char %02X, ignore message\n", c);
      state = MSG_START;
    }
  }
  else if (state == MSG_MESSAGE)
  {
    if (c == DLE)
    {
      state = MSG_ESCAPE;
    }
    else
    {
      *head++ = c;
    }
  }
  else
  {
    if (c == DLE)
    {
      state = MSG_ESCAPE;
    }
  }
}

void  f_ngt1::messageReceived(const unsigned char * msg, size_t msgLen)
{
  unsigned char command;
  unsigned char checksum = 0;
  unsigned char * payload;
  unsigned char payloadLen;
  size_t i;

  if (msgLen < 3)
  {
    printf("Ignore short command len = %zu\n", msgLen);
    return;
  }

  for (i = 0; i < msgLen; i++)
  {
    checksum += msg[i];
  }
  if (checksum)
  {
    printf("Ignoring message with invalid checksum\n");
    return;
  }

  command = msg[0];
  payloadLen = msg[1];

  if (command == N2K_MSG_RECEIVED)
  {
    n2kMessageReceived(msg + 2, payloadLen);
  }
  else if (command == NGT_MSG_RECEIVED)
  {
    ngtMessageReceived(msg + 2, payloadLen);
  }
}

void  f_ngt1::ngtMessageReceived(const unsigned char * msg, size_t msgLen)
{
  size_t i;
  char line[1000];
  char * p;
  char dateStr[DATE_LENGTH];

  if (msgLen < 12)
  {
    printf("Ignore short msg len = %zu\n", msgLen);
    return;
  }

  sprintf(line, "%s,%u,%u,%u,%u,%u", now(dateStr), 0, 0x40000 + msg[0], 0, 0, (unsigned int) msgLen - 1);
  p = line + strlen(line);
  for (i = 1; i < msgLen && p < line + sizeof(line) - 5; i++)
  {
    sprintf(p, ",%02x", msg[i]);
    p += 3;
  }
  *p++ = 0;
 
  if(showTxt){
    puts(line);
    fflush(stdout);
  }
}

void  f_ngt1::n2kMessageReceived(const unsigned char * msg, size_t msgLen)
{
  unsigned int prio, src, dst;
  unsigned int pgn;
  size_t i;
  unsigned int id;
  unsigned int len;
  unsigned int data[8];
  /*
  char line[800];
  char * p;
  */
  char dateStr[DATE_LENGTH];

  if (msgLen < 11)
  {
    printf("Ignoring N2K message - too short\n");
    return;
  }
  prio = msg[0];
  pgn  = (unsigned int) msg[1] + 256 * ((unsigned int) msg[2] + 256 * (unsigned int) msg[3]);
  dst  = msg[4];
  src  = msg[5];
  /* Skip the timestamp logged by the NGT-1-A in bytes 6-9 */
  len  = msg[10];

  if (len > 223)
  {
    printf("Ignoring N2K message - too long (%u)\n", len);
    return;
  }

  /*
  p = line;

  snprintf(p, sizeof(line), "%s,%u,%u,%u,%u,%u", now(dateStr), prio, pgn, src, dst, len);
  p += strlen(line);
  */
  msg_raw.prio = prio;
  msg_raw.pgn = pgn;
  msg_raw.dst = dst;
  msg_raw.src = src;
  msg_raw.len = len;
  
  snprintf(msg_raw.timestamp, sizeof(msg_raw.timestamp),
	   "%s", now(dateStr));
  
  /*
  len += 11;
  for (i = 11; i < len; i++)
  {
    
    snprintf(p, line + sizeof(line) - p, ",%02x", msg[i]);
    p += strlen(p);
  }
  */

  int idata=0;
  for(i = 11; idata < len; i++,idata++){
    msg_raw.data[idata] = msg[i];
  }

  /*
  puts(line);
  fflush(stdout);
  */

  /* decode msg_raw */
  printCanFormat(&msg_raw);
  printCanRaw(&msg_raw);
}


////////////////////////////////////////////////////////// from canboat.analyze

bool f_ngt1::printCanFormat(RawMessage * msg)
{
  size_t i;
  size_t unknownIndex = 0;

  if (onlySrc >=0 && onlySrc != msg->src)
  {
    return false;
  }

  for (i = 0; i < pgnListSize; i++)
  {
    if (msg->pgn == pgnList[i].pgn)
    {
      if (onlyPgn)
      {
        if (msg->pgn == onlyPgn)
        {
          printPacket(i, msg);
          return true;
        }
        continue;
      }
      if (!pgnList[i].size)
      {
        return true; /* We have field names, but no field sizes. */
      }
      /*
       * Found the pgn that matches this particular packet
       */
      printPacket(i, msg);
      return true;
    }
    else if (msg->pgn < pgnList[i].pgn)
    {
      break;
    }
    if (pgnList[i].unknownPgn)
    {
      unknownIndex = i;
    }
  }
  if (!onlyPgn)
  {
    printPacket(unknownIndex, msg);
  }
  return onlyPgn > 0;
}


void f_ngt1::printPacket(size_t index, RawMessage * msg)
{
  size_t fastPacketIndex;
  size_t bucket;
  Packet * packet;
  Pgn * pgn = &pgnList[index];
  size_t subIndex;

  if (!device[msg->src])
  {
    device[msg->src] = new DevicePackets;
    heapSize += sizeof(DevicePackets);
    heapSize += sizeof(Packet)*pgnListSize;    
    logDebug("New device at address %u (heap %zu bytes)\n", msg->src, heapSize);
    
    if (!device[msg->src])
    {
      die("Out of memory\n");
    }
  }
  packet = &(device[msg->src]->packetList[index]);

  if (!packet->data)
  {
    packet->allocSize = max(max(pgn->size, (uint32_t)8) + FASTPACKET_BUCKET_N_SIZE, (unsigned int) msg->len);
    heapSize += packet->allocSize;
    //logInfo("New PGN %u for device %u (heap %zu bytes)\n", pgn->pgn, msg->src, heapSize);
    packet->data = (uint8_t*)malloc(packet->allocSize);
    if (!packet->data)
    {
      die("Out of memory\n");
    }
  }

  if (msg->len > 0x8/*|| format != RAWFORMAT_PLAIN*/)
  {
    if (packet->allocSize < msg->len)
    {
      heapSize += msg->len - packet->allocSize;
      logDebug("Resizing buffer for PGN %u device %u to accomodate %u bytes (heap %zu bytes)\n", pgn->pgn, msg->src, msg->len, heapSize);
      packet->data = (uint8_t*)realloc(packet->data, msg->len);
      if (!packet->data)
      {
        die("Out of memory\n");
      }
      packet->allocSize = msg->len;
    }
    memcpy( packet->data
          , msg->data
          , msg->len
          );
    packet->size = msg->len;
  }
  else if (pgn->size > 0x8)
  {
    fastPacketIndex = msg->data[FASTPACKET_INDEX];
    bucket = fastPacketIndex & FASTPACKET_MAX_INDEX;

    if (bucket == 0)
    {
      size_t newSize = ((size_t) msg->data[FASTPACKET_SIZE]) + FASTPACKET_BUCKET_N_SIZE;

      if (packet->allocSize < newSize)
      {
        heapSize += newSize - packet->allocSize;
        logDebug("Resizing buffer for PGN %u device %u to accomodate %zu bytes (heap %zu bytes)\n", pgn->pgn, msg->src, newSize, heapSize);
        packet->data = (uint8_t*)realloc(packet->data, newSize);
        if (!packet->data)
        {
          die("Out of memory\n");
        }
        packet->allocSize = newSize;
      }
      packet->size = msg->data[FASTPACKET_SIZE];
      memcpy( packet->data
            , msg->data + FASTPACKET_BUCKET_0_OFFSET
            , FASTPACKET_BUCKET_0_SIZE
            );
    }
    else
    {
      if (packet->lastFastPacket + 1 != fastPacketIndex)
      {
        logError("PGN %u malformed packet for %u received; expected %zu but got %zu\n"
                , pgn->pgn, msg->src, packet->lastFastPacket + 1, fastPacketIndex
                );
        return;
      }
      memcpy( packet->data + FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * (bucket - 1)
            , msg->data + FASTPACKET_BUCKET_N_OFFSET
            , FASTPACKET_BUCKET_N_SIZE
            );
    }
    packet->lastFastPacket = fastPacketIndex;

    if (FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * bucket < packet->size)
    {
      /* Packet is not complete yet */
      return;
    }
  }
  else /* msg->len <= 8 && pgn->size <= 0x8 */
  {
    packet->size = msg->len;
    memcpy( packet->data
          , msg->data
          , msg->len
          );
  }

  printPgn(msg, packet->data, packet->size);
}

bool f_ngt1::printPgn(RawMessage* msg, uint8_t *dataStart, int length)
{
  Pgn *pgn;

  uint8_t * data;

  uint8_t * dataEnd = dataStart + length;
  size_t i;
  Field field;
  size_t bits;
  size_t bytes;
  size_t startBit;
  int      repetition = 0;
  uint16_t valueu16;
  uint32_t valueu32;
  uint16_t currentDate = UINT16_MAX;
  uint32_t currentTime = UINT32_MAX;
  char fieldName[60];
  bool r;
  uint32_t refPgn = 0;
  uint32_t variableFieldCount[2]; // How many variable fields over all repetitions, indexed by group
  uint32_t variableFields[2];     // How many variable fields per repetition, indexed by group
  size_t variableFieldStart;
  
  if (!msg) {
    return false;
  }
  pgn  = getMatchingPgn(msg->pgn, dataStart, length);
  if (!pgn) {
    pgn = pgnList;
  }
  
  PgnFieldValues * pfv = new PgnFieldValues(pgn);
  if(!pfv){
    cerr << "Failed to allocate memory" << endl;
    return false;
  }
  
  if (showData)
  {
    FILE * f = stdout;
    char c = ' ';

    if (showJson)
    {
      f = stderr;
    }

    fprintf(f, "%s %u %3u %3u %6u %s: ",
	    msg->timestamp, msg->prio, msg->src, msg->dst,
	    msg->pgn, pgn->description);
    for (i = 0; i < length; i++)
    {
      fprintf(f, " %2.02X", dataStart[i]);
    }
    putc('\n', f);

    fprintf(f, "%s %u %3u %3u %6u %s: ",
	    msg->timestamp, msg->prio, msg->src, msg->dst,
	    msg->pgn, pgn->description);
    
    for (i = 0; i < length; i++)
    {
      fprintf(f, "  %c", isalnum(dataStart[i]) ? dataStart[i] : '.');
    }
    putc('\n', f);
  }

  if(showTxt){
    if (showJson){
      if (pgn->camelDescription){
	mprintf("\"%s\":", pgn->camelDescription);
      }
      mprintf("{\"timestamp\":\"%s\",\"prio\":%u,\"src\":%u,\"dst\":%u,\"pgn\":%u,\"description\":\"%s\"", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
      strcpy(closingBraces, "}");
      sep = ",\"fields\":{";
    }else{
      mprintf("%s %u %3u %3u %6u %s:", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
      sep = " ";
    }
  }
  
  g_variableFieldRepeat[0] = 0;
  g_variableFieldRepeat[1] = 0;
  g_variableFieldIndex = 0;
  
  // Matsumoto Memo
  // in repeatingFields, properties of the two variable fileds are encoded; one is in less than 100, and another is in larger than 99
  // Currently,  list of parameter-value pairs are denoted as 2, and a list of parameters denoted as 1
  // meanings of following repeatingFields 
  // 1: a list of parameters
  // 2: a list of parameter-value pairas
  // 201: a list of parameters, a list of parameter-value pairs
  // 102: a list of parameter-value pairs, a list of parameters
  // 202: two lists of parameter value pairs
  // but I think second variable field cannot be processed properly...
  
  if (pgn->repeatingFields >= 100)
  {
    variableFieldCount[0] = pgn->repeatingFields % 100;
    variableFieldCount[1] = pgn->repeatingFields / 100;
  }
  else
  {
    variableFieldCount[0] = pgn->repeatingFields % 100;
    variableFieldCount[1] = 0;
  }
  variableFieldStart = pgn->fieldCount - variableFieldCount[0] - variableFieldCount[1];
  logDebug("fieldCount=%d variableFieldStart=%d\n", pgn->fieldCount, variableFieldStart);

  for (i = 0, startBit = 0, data = dataStart; data < dataEnd; i++)
  {
    Field* field;
    r = true;

    if (variableFieldCount[0] && i == variableFieldStart && repetition == 0)
    {
      // matsumoto memo
      // the codes assume that the g_variableFieldRepeat[0](and[1](if exists))
      // has already been filled by field "# of ..." previously called printNumber
      repetition = 1;
      if (showTxt && showJson)
      {
        mprintf("%s\"list\":[{", getSep());
        strcat(closingBraces, "]}");
        sep = "";
      }
      // Only now is g_variableFieldRepeat set via values for parameters starting with '# of ...'
      variableFields[0] = variableFieldCount[0] * g_variableFieldRepeat[0];
      variableFields[1] = variableFieldCount[1] * g_variableFieldRepeat[1];
    }
    if (repetition)
    {
      if (variableFields[0])
      {
        if (showTxt && showBytes)
        {
          mprintf("\ni=%d fs=%d vf=%d", i, variableFieldStart, variableFields[0]);
        }
	
        if (i == variableFieldStart + variableFieldCount[0])
        {
	// matsumoto memo
	// fields in variableFieldStart to variableFieldStart + variableFieldCount[0] - 1
	// should be repeated g_variableFieldRepeat[0] times.
          i = variableFieldStart;
          repetition++;
          if (showTxt && showJson)
          {
            mprintf("},{");
            sep = "";
          }
        }
        variableFields[0]--;
        if (variableFields[0] == 0)
        {
	  // matsumoto memo
	  // At the end of the repetiion, variableFieldStart is updated
	  // to point the second variableField if exists.
          variableFieldStart += variableFieldCount[0];
        }
      }
      else if (variableFields[1])
      {
        if (variableFields[1] == variableFieldCount[1] * g_variableFieldRepeat[1])
        {
	  // matumoto memo
	  // The first entrance, repetition reset?
	  // it won't work because variableFields[0] is non-zero in later iteration.
          repetition = 0;
        }
        if (i == variableFieldStart + variableFieldCount[1])
        {
	  //matumoto memo
	  // To repeate the second variable fields. The same code for the first variable fields is found in previous lines.
          i = variableFieldStart;
          repetition++;
          if (showTxt && showJson)
          {
            mprintf("},{");
            sep = "";
          }
        }
        variableFields[1]--;
      }
      else
      {
        break;
      }
    }

    field = &pgn->fieldList[i];

    if (!field->camelName && !field->name)
    {
      logDebug("PGN %u has unknown bytes at end: %u\n", msg->pgn, dataEnd - data);
      break;
    }

    strcpy(fieldName, field->camelName ? field->camelName : field->name);
    if (repetition >= 1 && !showJson)
    {
      strcat(fieldName, field->camelName ? "_" : " ");
      sprintf(fieldName + strlen(fieldName), "%u", repetition);
    }

    bits  = field->size;
    bytes = (bits + 7) / 8;
    bytes = min(bytes, (size_t) (dataEnd - data));
    bits  = min(bytes * 8, bits);

    if (showTxt && showBytes)
    {
      mprintf("\ndecode %s offset=%u startBit=%u bits=%u bytes=%u:", field->name, data - dataStart, startBit, bits, bytes);
    }

    if (strcmp(fieldName, "PGN") == 0)
    {
      refPgn = data[0] + (data[1] << 8) + (data[2] << 16);
      if (showTxt && showBytes)
      {
        mprintf("refPgn=%u ", refPgn);
      }
    }

    if (strcmp(fieldName, "Reserved") == 0)
    {
      // Skipping reserved fields. Unfortunately we have some cases now
      // where they are zero. Some AIS devices (SRT) produce an 8 bit long
      // nav status, others just a four bit one.
    }
    else if (field->resolution < 0.0)
    {
      int len;
      int k;

      bool ascii = false;
      /* These fields have only been found to start on byte boundaries,
       * making their location easier
       */
      if (field->resolution == RES_STRINGLZ)
      {
        len = *data++;
        bytes--;
	ascii = true;
      }else if (field->resolution == RES_STRINGLAU)
      {
        int control;

        len = *data++;
        bytes--;
        control = *data++;
        bytes--;
        if (control == 0)
        {
          logError("Unhandled UNICODE string in PGN\n");
        }
	ascii = true;
      }else if (field->resolution == RES_ASCII){
        len = (int) bytes;
        unsigned char lastbyte = data[len - 1];
	
        if (lastbyte == 0xff || lastbyte == ' ' || lastbyte == 0 || lastbyte == '@')
	  {
	    while (len > 0 && (data[len - 1] == lastbyte))
	      {
		len--;
	      }
	  }
	ascii = true;
      }

      if(ascii){
	string str; 
	for (k = 0; k < len; k++)
	  {
	    if (data[k] == 0xff)
	      {
		break;
	      }
	    if (data[k] >= ' ' && data[k] <= '~')
	      {
		str.push_back(data[k]); // store ascii string in data
	      }
	  }
	pfv->push(str); // push into field variable	  
	
	if(showTxt){
	  if (showBytes)
	    {
	      for (k = 0; k < len; k++)
		{
		  mprintf("%02x ", data[k]);
		}
	    }	  
	  if (showJson)
	    {
	      mprintf("%s\"%s\":\"", getSep(), fieldName);
	      print_ascii_json_escaped(data, len);
	      mprintf("\"");
	    }
	  else
	    {
	      mprintf("%s %s = ", getSep(), fieldName);
	      mprintf("%s", str.c_str());
	    }
	}
      }
      
      if (field->resolution == RES_STRING)
	{
	  int len;
	  // there maybe two modes
	  // one is string starting with 0x02 and terminating with 0x01
	  // another is starting with length larger than 0x2
	  if (*data == 0x02){ // 0x02 is the starting code
	    // counting length 
	    data++;
	    for (len = 0;
		 data + len < dataEnd
		   && data[len] != 0x01; len++);
	    bytes = len + 1;
	  }
	  else if (*data > 0x02){
	    bytes = *data++;
	    bytes--; /* Compensate for that we've already increased data by 1 */
	    if (*data == 0x01)
	      {
		data++;
		bytes--;
	      }
	    len = bytes - 1;
	  }
	  else{
	    bytes = 1;
	    len = 0;
	  }
	  
	  if (len){
	    string str;
	    for (int idata = 0; idata < len; idata++)
	      str.push_back(data[idata]);
	    pfv->push(str);

	    if(showTxt){
	      if (showJson){
		mprintf("%s\"%s\":\"%.*s\"", getSep(), fieldName, (int) len, data);
	      }
	      else{
		mprintf("%s %s = %.*s", getSep(), fieldName, (int) len, data);
	      }
	    }
	  }
	  bits = BYTES(bytes);
	}
      else if (field->resolution == RES_LONGITUDE || field->resolution == RES_LATITUDE)
	{
	  printLatLon(fieldName, field->resolution, data, bytes, pfv);
	}
      else if (field->resolution == RES_DATE)
	{
	  //GMT days from (1/1/1970) in two bytes
	  memcpy((void *) &valueu16, data, 2);
	  pfv->push(valueu16);
	  if(showTxt)
	    printDate(fieldName, valueu16);
	  
	  currentDate = valueu16;
	}
      else if (field->resolution == RES_TIME)
	{
	  // 100usec in 4 bytes
	  memcpy((void *) &valueu32, data, 4);
	  pfv->push(valueu32);
	  if(showTxt)
	    printTime(fieldName, valueu32);
	  currentTime = valueu32;
	}
      else if (field->resolution == RES_PRESSURE)
	{
	  valueu32 = data[0] + (data[1] << 8);
	  printPressure(fieldName, valueu32, field, pfv);
	}
      else if (field->resolution == RES_PRESSURE_HIRES)
	{
	  valueu32 = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
	  printPressure(fieldName, valueu32, field, pfv);
	}
      else if (field->resolution == RES_TEMPERATURE)
	{
	  valueu32 = data[0] + (data[1] << 8);
	  printTemperature(fieldName, valueu32, 16, 0.01, pfv);
	}
      else if (field->resolution == RES_TEMPERATURE_HIGH)
	{
	  valueu32 = data[0] + (data[1] << 8);
	  printTemperature(fieldName, valueu32, 16, 0.1, pfv);
	}
      else if (field->resolution == RES_TEMPERATURE_HIRES)
	{
	  valueu32 = data[0] + (data[1] << 8) + (data[2] << 16);
	  printTemperature(fieldName, valueu32, 24, 0.001, pfv);
	}
      else if (field->resolution == RES_6BITASCII)
	{
	  // is not used.
	  print6BitASCIIText(fieldName, data, startBit, bits);
	}
      else if (field->resolution == RES_DECIMAL)
	{
	  printDecimal(fieldName, data, startBit, bits, pfv);
	}
      else if (bits == LEN_VARIABLE)
	{
	  printVarNumber(fieldName, pgn, refPgn, field, data, startBit, &bits, pfv);
	}
      else if (bits > BYTES(8))
	{
	  printHex(fieldName, data, startBit, bits, pfv);
	}
      else if (field->resolution == RES_INTEGER
	       || field->resolution == RES_LOOKUP
	       || field->resolution == RES_BITFIELD
	       || field->resolution == RES_BINARY
	       || field->resolution == RES_MANUFACTURER
	       )
	{
	  printNumber(fieldName, field, data, startBit, bits, pfv);
	}
      else
	{
	  logError("Unknown resolution %f for %s\n", field->resolution, fieldName);
	}
    }
    else if (field->resolution > 0.0)
      {
	printNumber(fieldName, field, data, startBit, bits, pfv);
      }
    if (!r)
      {
	return false;
      }
    
    startBit += bits;
    data += startBit / 8;
    startBit %= 8;
  }

  if(showTxt){
    if (showJson)
      {
	for (i = strlen(closingBraces); i;)
	  {
	    mprintf("%c", closingBraces[--i]);
	  }
      }
    mprintf("\n");
    if (r){
      mwrite(stdout);
    }
    else{
      mreset();
    }    
  }
  
  /*
  if (msg->pgn == 126992 && currentDate < UINT16_MAX && currentTime < UINT32_MAX && clockSrc == msg->src)
    {
      setSystemClock(currentDate, currentTime);
    }
  */
  pgn_queue.push_back(pfv);
  return r;
}

void f_ngt1::printCanRaw(RawMessage * msg)
{
  size_t i;
  FILE * f = stdout;

  if (onlySrc >= 0 && onlySrc != msg->src)
  {
    return;
  }

  if (showJson)
  {
    f = stderr;
  }

  if (showRaw && (!onlyPgn || onlyPgn == msg->pgn))
  {
    fprintf(f, "%s %u %03u %03u %6u :", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn);
    for (i = 0; i < msg->len; i++)
    {
      fprintf(f, " %02x", msg->data[i]);
    }
    putc('\n', f);
  }
}


const char * f_ngt1::getSep()
{
  const char * s = sep;

  if (showJson)
  {
    sep = ",";
    if (strchr(s, '{'))
    {
      if (strlen(closingBraces) >= sizeof(closingBraces) - 2)
      {
        logError("Too many braces\n");
        exit(2);
      }
      strcat(closingBraces, "}");
    }
  }
  else
  {
    sep = ";";
  }

  return s;
}

void f_ngt1::mprintf(const char * format, ...)
{
  va_list ap;
  int remain;

  va_start(ap, format);
  remain = sizeof(mbuf) - (mp - mbuf) - 1;
  if (remain > 0) {
    mp += vsnprintf(mp, remain, format, ap);
  }
  va_end(ap);
}


void f_ngt1::mreset(void)
{
  mp = mbuf;
}

void f_ngt1::mwrite(FILE * stream)
{
  fwrite(mbuf, sizeof(char), mp - mbuf, stream);
  fflush(stream);
  mreset();
}


bool f_ngt1::printLatLon(char * name, double resolution, uint8_t * data, size_t bytes, PgnFieldValues * pfv)
{
  uint64_t absVal;
  int64_t value;

  value = 0;
  memcpy(&value, data, bytes);
  if (bytes == 4 && ((data[3] & 0x80) > 0))
  {
    value |= UINT64_C(0xffffffff00000000);
  }
  if (value > ((bytes == 8) ? INT64_C(0x7ffffffffffffffd) : INT64_C(0x7ffffffd)))
  {
    return false;
  }

  if (bytes == 8)
  {
    if (showTxt && showBytes)
    {
      mprintf("(%" PRIx64 " = %" PRId64 ") ", value, value);
    }

    value /= INT64_C(1000000000);
  }
  absVal = (value < 0) ? -value : value;

  if (showTxt && showBytes)
  {
    mprintf("(%" PRId64 ") ", value);
  }

  double dd = (double) value / (double) RES_LAT_LONG_PRECISION;
  pfv->push(dd);

  if(showTxt){
    if (showGeo == GEO_DD)
      {
	
	if (showJson){
	  mprintf("%s\"%s\":%10.7f", getSep(), name, dd);
	}
	else{
	  mprintf("%s %s = %10.7f", getSep(), name, dd);
	}
      }
    else if (showGeo == GEO_DM)
      {
	/* One degree = 10e6 */
	
	uint64_t degrees = (absVal / RES_LAT_LONG_PRECISION);
	uint64_t remainder = (absVal % RES_LAT_LONG_PRECISION);
	double minutes = (remainder * 60) / (double) RES_LAT_LONG_PRECISION;
	
	if(showJson){
	  mprintf("%s\"%s\":\"%02u&deg; %6.3f %c\""
		  , getSep(), name, (uint32_t) degrees, minutes
		  , ((resolution == RES_LONGITUDE)
		     ? ((value >= 0) ? 'E' : 'W')
		     : ((value >= 0) ? 'N' : 'S')
		     )
		  );
	  
	}else{
	  mprintf("%s %s = %02ud %6.3f %c"
		  , getSep(), name, (uint32_t) degrees, minutes
		  , ((resolution == RES_LONGITUDE)
		     ? ((value >= 0) ? 'E' : 'W')
		     : ((value >= 0) ? 'N' : 'S')
		     )
		  );
	  
	}
      }
    else{
      uint32_t degrees = (uint32_t) (absVal / RES_LAT_LONG_PRECISION);
      uint32_t remainder = (uint32_t) (absVal % RES_LAT_LONG_PRECISION);
      uint32_t minutes = (remainder * 60) / RES_LAT_LONG_PRECISION;
      double seconds = (((uint64_t) remainder * 3600) / (double) RES_LAT_LONG_PRECISION) - (60 * minutes);
      
      if(showJson){
	mprintf("%s\"%s\":\"%02u&deg;%02u&rsquo;%06.3f&rdquo;%c\""
		, getSep(), name, degrees, minutes, seconds
		, ((resolution == RES_LONGITUDE)
		   ? ((value >= 0) ? 'E' : 'W')
		   : ((value >= 0) ? 'N' : 'S')
		   )
		);
	
      }else{
	mprintf( "%s %s = %02ud %02u' %06.3f\"%c"
		 , getSep(), name, degrees, minutes, seconds
		 , ((resolution == RES_LONGITUDE)
		    ? ((value >= 0) ? 'E' : 'W')
		    : ((value >= 0) ? 'N' : 'S')
		    )
		 );       
      }
      
      if (showJson){
	double dd = (double) value / (double) RES_LAT_LONG_PRECISION;
	mprintf("%s\"%s_dd\":%10.7f", getSep(), name, dd);
      }
    }
  }
  return true;
}


bool f_ngt1::printDate(char * name, uint16_t d)
{
  char buf[sizeof("2008.03.10") + 1];
  time_t t;
  struct tm * tm;

  if (d >= 0xfffd)
  {
    return false;
  }

  if (showBytes)
  {
    mprintf("(date %hx = %hd) ", d, d);
  }

  t = d * 86400;
  tm = gmtime(&t);
  if (!tm)
  {
    logAbort("Unable to convert %u to gmtime\n", (unsigned int) t);
  }
  strftime(buf, sizeof(buf), "%Y.%m.%d", tm);
  if (showJson)
  {
    mprintf("%s\"%s\":\"%s\"", getSep(), name, buf);
  }
  else
  {
    ///////////////////////////////////////////////////////// insert data fetching code here
    mprintf("%s %s = %s", getSep(), name, buf);
  }
  return true;
}

bool f_ngt1::printTime(char * name, uint32_t t)
{
  uint32_t hours;
  uint32_t minutes;
  uint32_t seconds;
  uint32_t units;
  const uint32_t unitspersecond = 10000;

  if (t >= 0xfffffffd)
  {
    return false;
  }

  if (showBytes)
  {
    mprintf("(time %x = %u) ", t, t);
  }


  seconds = t / unitspersecond;
  units = t % unitspersecond;
  minutes = seconds / 60;
  seconds = seconds % 60;
  hours = minutes / 60;
  minutes = minutes % 60;

  if (showJson)
  {
    if (units)
    {
      mprintf("%s \"%s\": \"%02u:%02u:%02u.%05u\"", getSep(), name, hours, minutes, seconds, units);
    }
    else
    {
      mprintf("%s \"%s\": \"%02u:%02u:%02u\"", getSep(), name, hours, minutes, seconds);
    }
  }
  else
  {
    ///////////////////////////////////////////////////////// insert data fetching code here
    if (units)
    {
      mprintf("%s %s = %02u:%02u:%02u.%05u", getSep(), name, hours, minutes, seconds, units);
    }
    else
    {
      mprintf("%s %s = %02u:%02u:%02u", getSep(), name, hours, minutes, seconds);
    }
  }
  return true;
}


bool f_ngt1::printTemperature(char * name, uint32_t t, uint32_t bits,
			      double resolution, PgnFieldValues * pfv)
{
  double k = t * resolution;
  pfv->push(k);
  if(showTxt){
    double c = k - 273.15;
    double f = c * 1.8 + 32;
    
    if ((bits == 16 && t >= 0xfffd) || (bits == 24 && t >= 0xfffffd)){
      return false;
    }
    
    if (showSI){
      if (showJson){
	mprintf("%s\"%s\":%.2f", getSep(), name, k, f);
      }
      else{
	mprintf("%s %s = %.2f K", getSep(), name, k);
      }
      return true;
    }
    
    if (showJson){
      mprintf("%s\"%s\":%.2f", getSep(), name, c, f);
    }
    else{
      mprintf("%s %s = %.2f C (%.1f F)", getSep(), name, c, f);
    }
  }
  return true;
}

bool f_ngt1::printPressure(char * name, uint32_t v, Field * field,
			   PgnFieldValues * pfv)
{
  int32_t pressure = 0;
  double bar;
  double psi;

  if (field->size <= 16)
  {
    if (v >= 0xfffd)
    {
      pfv->push(pressure);
      return false;
    }
  }
  if (v >= 0xfffffffd)
  {
    pfv->push(pressure);
    return false;
  }

  // There are four types of known pressure: unsigned hectopascal, signed kpa, unsigned kpa, unsigned four bytes in pascal.

  if (field->hasSign)
  {
    pressure = (int16_t) v;
  }
  else
  {
    pressure = v;
  }
  // Now scale pascal properly, it is in hPa or kPa.
  if (field->units)
  {
    switch (field->units[0])
    {
    case 'h':
    case 'H':
      pressure *= 100;
      break;
    case 'k':
    case 'K':
      pressure *= 1000;
      break;
    case 'd':
      pressure /= 10;
      break;
    }
  }
  
  pfv->push(pressure);


  if(showTxt){
    bar = pressure / 100000.0; /* 1000 hectopascal = 1 Bar */
    psi = pressure / 1450.377; /* Silly but still used in some parts of the world */
    
    if (showJson)
      {
#ifdef _WIN32
      mprintf("%s\"%s\":%" "ld" "", getSep(), name, pressure);
#else
	mprintf("%s\"%s\":%" PRId32 "", getSep(), name, pressure);
#endif
      }
    else
      {
	mprintf("%s %s = %.3f bar (%.1f PSI)", getSep(), name, bar, psi);
      }
  }
  return true;
}

void f_ngt1::print6BitASCIIChar(uint8_t b)
{
  int c;
  if (b < 0x28)
  {
    c = b + 0x30;
  }
  else
  {
    c = b + 0x38;
  }
  if (showJson && (c == '\\'))
  {
    putchar(c);
  }
  putchar(c);
}

bool f_ngt1::print6BitASCIIText(char * name, uint8_t * data, size_t startBit, size_t bits)
{
  uint8_t value = 0;
  uint8_t maxValue = 0;
  uint8_t bitMask = 1 << startBit;
  uint64_t bitMagnitude = 1;
  size_t bit;
  char buf[128];

  if (showJson){
    mprintf("%s\"%s\":\"", getSep(), name);
  }
  else{
    mprintf("%s %s = ", getSep(), name);
  }

  for (bit = 0; bit < bits && bit < sizeof(buf) * 8; bit++)
  {
    /* Act on the current bit */
    bool bitIsSet = (*data & bitMask) > 0;
    maxValue |= bitMagnitude;
    if (bitIsSet){
      value |= bitMagnitude;
    }

    /* Find the next bit */
    if (bitMask == 128){
      bitMask = 1;
      data++;
    }
    else{
      bitMask = bitMask << 1;
    }
    bitMagnitude = bitMagnitude << 1;

    if (bit % 6 == 5){
      print6BitASCIIChar(value);
      value = 0;
      bitMagnitude = 1;
    }
  }
  if (showJson){
    mprintf("\"");
  }
  return true;
}


bool f_ngt1::printHex(char * name, uint8_t * data, size_t startBit, size_t bits , PgnFieldValues * pfv)
{
  uint8_t value = 0;
  uint8_t maxValue = 0;
  uint8_t bitMask = 1 << startBit;
  uint64_t bitMagnitude = 1;
  size_t bit;
  char buf[128];

  string hex;
  char hexstr[4];
  for (bit = 0; bit < bits && bit < sizeof(buf) * 8; bit++)
  {
    /* Act on the current bit */
    bool bitIsSet = (*data & bitMask) > 0;
    maxValue |= bitMagnitude;
    if (bitIsSet){
      value |= bitMagnitude;
    }

    /* Find the next bit */
    if (bitMask == 128){
      bitMask = 1;
      data++;
    }
    else{
      bitMask = bitMask << 1;
    }
    bitMagnitude = bitMagnitude << 1;

    if (bit % 8 == 7){
      sprintf(hexstr, "%02x ", value);
      hex += hexstr;
      value = 0;
      bitMagnitude = 1;
    }
  }

  if(showTxt){
    if (showBytes){
      mprintf("(%s,%p,%zu,%zu) ", name, data, startBit, bits);
    }
    
    if (showJson){
      mprintf("%s\"%s\":\"", getSep(), name);
    }
    else{
      mprintf("%s %s = ", getSep(), name);
    }
    mprintf("%s", hex.c_str());
    if (showJson){
      mprintf("\"");
    }    
  }
  pfv->push(hex);
  
  return true;
}


bool f_ngt1::printDecimal(char * name, uint8_t * data, size_t startBit, size_t bits, PgnFieldValues * pfv)
{
  uint8_t value = 0;
  uint8_t maxValue = 0;
  uint8_t bitMask = 1 << startBit;
  uint64_t bitMagnitude = 1;
  size_t bit;
  char buf[128];

  unsigned long long result = 0;
  string dec;
  char dec_str[3];
  for (bit = 0; bit < bits && bit < sizeof(buf) * 8; bit++)
  {
    /* Act on the current bit */
    bool bitIsSet = (*data & bitMask) > 0;
    maxValue |= bitMagnitude;
    if (bitIsSet)
    {
      value |= bitMagnitude;
    }

    /* Find the next bit */
    if (bitMask == 128)
    {
      bitMask = 1;
      data++;
    }
    else
    {
      bitMask = bitMask << 1;
    }
    bitMagnitude = bitMagnitude << 1;

    if (bit % 8 == 7)
    {
      if (value < 100)
      {
	sprintf(dec_str, "%02u", value);
	dec += dec_str;
        mprintf("%02u", value);
      }
      value = 0;
      bitMagnitude = 1;
    }
  }
  pfv->push(dec);


  if(showTxt){
    if (showBytes){
      mprintf("(%s,%p,%zu,%zu) ", name, data, startBit, bits);
    }
    
    if (showJson){
      mprintf("%s\"%s\":\"", getSep(), name);
    }
    else{
      mprintf("%s %s = ", getSep(), name);
    }
    mprintf("%s", dec.c_str());
    if (showJson){
      mprintf("\"");
    }
  }
  return true;
}

bool f_ngt1::printVarNumber(char * fieldName, Pgn * pgn, uint32_t refPgn, Field * field, uint8_t * data, size_t startBit, size_t * bits, PgnFieldValues * pfv)
{
  Field * refField;
  size_t size, bytes;

  /* PGN 126208 contains variable field length.
   * The field length can be derived from the PGN mentioned earlier in the message,
   * plus the field number.
   */

  /*
   * This is rather hacky. We know that the 'data' pointer points to the n-th variable field
   * length and thus that the field number is exactly one byte earlier.
   */

  refField = getField(refPgn, data[-1] - 1);
  // retrieving bit width of the field from corresponding pgn 
  if (refField)
  {
    *bits = (refField->size + 7) & ~7; // Round # of bits in field refField up to complete bytes: 1->8, 7->8, 8->8 etc.
    if (showTxt && showBytes)
    {
      mprintf("(refField %s size = %u in %zu bytes)", refField->name, refField->size, *bits / 8);
    }
    return printNumber(fieldName, field, data, startBit, refField->size, pfv);
  }

  logError("Pgn %d Field %s: cannot derive variable length from PGN %d field # %d\n"
          , pgn->pgn, field->name, refPgn, data[-1]);
  *bits = 8; /* Gotta assume something */
  return false;
}

bool f_ngt1::printNumber(char * fieldName, Field * field, uint8_t * data, size_t startBit, size_t bits, PgnFieldValues * pfv)
{
  bool ret = false;
  int64_t value;
  int64_t maxValue;
  int64_t reserved;
  double a;

  extractNumber(field, data, startBit, bits, &value, &maxValue);
  pfv->push(value);

  /* There are max five reserved values according to ISO 11873-9 (that I gather from indirect sources)
   * but I don't yet know which datafields reserve the reserved values.
   */
#define DATAFIELD_UNKNOWN   (0)
#define DATAFIELD_ERROR     (-1)
#define DATAFIELD_RESERVED1 (-2)
#define DATAFIELD_RESERVED2 (-3)
#define DATAFIELD_RESERVED3 (-4)

  if (maxValue >= 15)
  {
    reserved = 2; /* DATAFIELD_ERROR and DATAFIELD_UNKNOWN */
  }
  else if (maxValue > 1)
  {
    reserved = 1; /* DATAFIELD_UNKNOWN */
  }
  else
  {
    reserved = 0;
  }

  if (fieldName[0] == '#')
  {
    logDebug("g_variableFieldRepeat[%d]=%d\n", g_variableFieldIndex, value);
    g_variableFieldRepeat[g_variableFieldIndex++] = value;
  }

  if(!showTxt)
    return true;

  if (value <= maxValue - reserved)
  {
    if (field->units && field->units[0] == '=')
    {
      char lookfor[20];
      char * s;

      sprintf(lookfor, "=%" PRId64 , value);
      if (strcmp(lookfor, field->units) != 0)
      {
        if (showBytes) logError("Field %s value %" PRId64 " does not match %s\n", fieldName, value, field->units + 1);
        return false;
      }
      s = field->description;
      if (!s)
      {
        s = lookfor + 1;
      }
      if (showJson)
      {
        mprintf("%s\"%s\":\"%s\"", getSep(), fieldName, s);
      }
      else
      {
	///////////////////////////////////////////////////////// insert data fetching code here    
        mprintf("%s %s = %s", getSep(), fieldName, s);
      }
    }

    else if (field->resolution == RES_LOOKUP && field->units)
    {
      char lookfor[20];
      char * s, * e;

      sprintf(lookfor, ",%" PRId64 "=", value);
      s = strstr(field->units, lookfor);
      if (s)
      {
        s += strlen(lookfor);
        e = strchr(s, ',');
        e = e ? e : s + strlen(s);
        if (showJsonValue)
        {
          mprintf("%s\"%s\":{\"value\":%" PRId64 ",\"name\":\"%.*s\"}", getSep(), fieldName, value, (int) (e - s), s);
        }
        else if (showJson)
        {
          mprintf("%s\"%s\":\"%.*s\"", getSep(), fieldName, (int) (e - s), s);
        }
        else
        {
	  ///////////////////////////////////////////////////////// insert data fetching code here    
          mprintf("%s %s = %.*s", getSep(), fieldName, (int) (e - s), s);
        }
      }
      else
      {
        if (showJson)
        {
          mprintf("%s\"%s\":\"%" PRId64 "\"", getSep(), fieldName, value);
        }
        else
        {
	  ///////////////////////////////////////////////////////// insert data fetching code here    
          mprintf("%s %s = %" PRId64 "", getSep(), fieldName, value);
        }
      }
    }

    else if (field->resolution == RES_BITFIELD && field->units)
    {
      char lookfor[20];
      char * s, * e;
      unsigned int bit;
      uint64_t bitValue;
      char sep;

      logDebug("RES_BITFIELD value %" PRIx64 "\n", value);
      if (showJson)
      {
        mprintf("%s\"%s\": ", getSep(), fieldName);
        sep = '[';
      }
      else
      {
	///////////////////////////////////////////////////////// insert data fetching code here    
        mprintf("%s %s =", getSep(), fieldName);
        sep = ' ';
      }

      for (bitValue = 1, bit = 0; bitValue <= maxValue; (bitValue *= 2), bit++)
      {
        logDebug("RES_BITFIELD is bit %u value %" PRIx64 " set %d\n", bit, bitValue, (value & value) >= 0);
        if ((value & bitValue) != 0)
        {
          sprintf(lookfor, ",%u=", bit);
          s = strstr(field->units, lookfor);
          if (s)
          {
            s += strlen(lookfor);
            e = strchr(s, ',');
            e = e ? e : s + strlen(s);
            if (showJson)
            {
              mprintf("%c\"%.*s\"", sep, (int) (e - s), s);
              sep = ',';
            }
            else
            {
	      ///////////////////////////////////////////////////////// insert data fetching code here    
              mprintf("%c%.*s", sep, (int) (e - s), s);
              sep = ',';
            }
          }
          else
          {
	    ///////////////////////////////////////////////////////// insert data fetching code here    
            mprintf("%c\"%" PRIu64 "\"", sep, bitValue);
            sep = ',';
          }
        }
      }
      if (showJson)
      {
        if (sep != '[')
        {
          mprintf("]");
        }
        else
        {
          mprintf("[]");
        }
      }
    }

    else if (field->resolution == RES_BINARY)
    {
      if (showJson)
      {
        mprintf("%s\"%s\":\"%" PRId64 "\"", getSep(), fieldName, value);
      }
      else
      {
	///////////////////////////////////////////////////////// insert data fetching code here    
        mprintf("%s %s = 0x%" PRIx64 , getSep(), fieldName, value);
      }
    }
    else if (field->resolution == RES_MANUFACTURER)
    {
      const char * m = 0;
      char unknownManufacturer[30];

      if (value > 0 && value < ARRAY_SIZE(manufacturer))
      {
        m = manufacturer[value];
      }
      if (!m)
      {
        if (showJson)
        {
          mprintf("%s \"%s\":%" PRId64 , getSep(), fieldName, value);
          return true;
        }
        sprintf(unknownManufacturer, "Unknown Manufacturer %" PRId64 , value);
        m = unknownManufacturer;
      }

      if (showJsonValue)
      {
        mprintf("%s \"%s\":{\"value\":%" PRId64 ",\"name\":\"%s\"}", getSep(), fieldName, value, m);
      }
      else if (showJson)
      {
        mprintf("%s \"%s\": \"%s\"", getSep(), fieldName, m);
      }
      else
      {
	///////////////////////////////////////////////////////// insert data fetching code here    
        mprintf("%s %s = %s", getSep(), fieldName, m);
      }
    }
    else
    {

      if (field->resolution == RES_INTEGER)
      {
        if (showJson)
        {
          mprintf("%s\"%s\":%" PRId64 "", getSep(), fieldName, value);
        }
        else
        {
	  ///////////////////////////////////////////////////////// insert data fetching code here    
          mprintf("%s %s = %" PRId64 , getSep(), fieldName, value);
        }
      }
      else
      {
        int precision;
        double r;
        const char * units = field->units;

        a = (double) value * field->resolution;

        precision = 0;
        for (r = field->resolution; (r > 0.0) && (r < 1.0); r *= 10.0)
        {
          precision++;
        }

        if (field->resolution == RES_RADIANS)
        {
          units = "rad";
          if (!showSI)
          {
            a *= RadianToDegree;
            precision -= 3;
            units = "deg";
          }
        }
        else if (field->resolution == RES_ROTATION || field->resolution == RES_HIRES_ROTATION)
        {
          units = "rad/s";
          if (!showSI)
          {
            a *= RadianToDegree;
            precision -= 3;
            units = "deg/s";
          }
        }
        else if (units && showSI)
        {
          if (strcmp(units, "kWh") == 0)
          {
            a *= 3.6e6; // 1 kWh = 3.6 MJ.
          }
          else if (strcmp(units, "Ah") == 0)
          {
            a *= 3600.0; // 1 Ah = 3600 C.
          }

          // Many more to follow, but pgn.h is not yet complete enough...
        }


        if (showJson)
        {
          mprintf("%s\"%s\":%.*f", getSep(), fieldName, precision, a);
        }
        else if (units && strcmp(units, "m") == 0 && a >= 1000.0)
        {
	  ///////////////////////////////////////////////////////// insert data fetching code here    
          mprintf("%s %s = %.*f km", getSep(), fieldName, precision + 3, a / 1000);
        }
        else
        {
	  ///////////////////////////////////////////////////////// insert data fetching code here    
          mprintf("%s %s = %.*f", getSep(), fieldName, precision, a);
          if (units)
          {
            mprintf(" %s", units);
          }
        }
      }
    }
  }
  else
  {
    /* For json, which is supposed to be effective for machine operations
     * we just ignore the special values.
     */
    if (!showJson)
    {
      ///////////////////////////////////////////////////////// insert data fetching code here    
      switch (value - maxValue)
      {
      case DATAFIELD_UNKNOWN:
        mprintf("%s %s = Unknown", getSep(), fieldName);
        break;
      case DATAFIELD_ERROR:
        mprintf("%s %s = ERROR", getSep(), fieldName);
        break;
      case DATAFIELD_RESERVED1:
        mprintf("%s %s = RESERVED1", getSep(), fieldName);
        break;
      case DATAFIELD_RESERVED2:
        mprintf("%s %s = RESERVED2", getSep(), fieldName);
        break;
      case DATAFIELD_RESERVED3:
        mprintf("%s %s = RESERVED3", getSep(), fieldName);
        break;
      default:
        mprintf("%s %s = Unhandled value %ld (%ld)", getSep(), fieldName, value, value - maxValue);
      }
    }
  }

  return true;
}


void f_ngt1::print_ascii_json_escaped(uint8_t *data, int len)
{
  int c;
  int k;

  for (k = 0; k < len; k++)
  {
    c = data[k];
    switch(c)
    {
      case '\b':
        mprintf("%s", "\\b");
        break;

      case '\n':
        mprintf("%s", "\\n");
        break;

      case '\r':
        mprintf("%s", "\\r");
        break;

      case '\t':
        mprintf("%s", "\\t");
        break;

      case '\f':
        mprintf("%s", "\\f");
        break;

      case '"':
        mprintf("%s", "\\\"");
        break;

      case '\\':
        mprintf("%s", "\\\\");
        break;

      case '/':
        mprintf("%s", "\\/");
        break;

      case '\377':
        // 0xff has been seen on recent Simrad VHF systems, and it seems to indicate
        // end-of-field, with noise following. Assume this does not break other systems.
        return;

      default:
        if (c >= ' ' && c <= '~')
          mprintf("%c", c);
    }
  }
}

void f_ngt1::setSystemClock(uint16_t currentDate, uint32_t currentTime)
{

#ifndef SKIP_SETSYSTEMCLOCK
  static uint16_t prevDate = UINT16_MAX;
  static uint32_t prevTime = UINT32_MAX;
  const uint32_t unitspersecond = 10000;
  const uint32_t microsperunit = 100;
  const uint32_t microspersecond = 1000000;
  const uint32_t secondsperday = 86400;
  struct timeval now;
  struct timeval gps;
  struct timeval delta;
  struct timeval olddelta;

#ifdef HAS_ADJTIME
  const int maxDelta = 30;
#else
  const int maxDelta = 1;
#endif

  logDebug("setSystemClock = %u/%u\n", currentDate, currentTime);

  if (prevDate == UINT16_MAX)
  {
    logDebug("setSystemClock: first time\n");
    prevDate = currentDate;
    prevTime = currentTime;
    return;
  }
  if (prevTime == currentTime && prevDate == currentDate)
  {
    logDebug("System clock not changed\n");
    return;
  }

  if (gettimeofday(&now, 0))
  {
    logError("Can't get system clock\n");
    return;
  }

  gps.tv_sec = currentDate * secondsperday + currentTime / unitspersecond;
  gps.tv_usec = (currentTime % unitspersecond) * microsperunit;

  if (gps.tv_sec < now.tv_sec - maxDelta || gps.tv_sec > now.tv_sec + maxDelta)
  {
    if (settimeofday(&gps, 0))
    {
      logError("Failed to adjust system clock to %" PRIu64 "/%06u\n", (uint64_t) gps.tv_sec, gps.tv_usec);
      return;
    }
    if (showBytes)
    {
      logInfo("Set system clock to %" PRIu64 "/%06u\n", (uint64_t) gps.tv_sec, gps.tv_usec);
    }
    return;
  }

#ifdef HAS_ADJTIME

  delta.tv_sec = 0;
  delta.tv_usec = gps.tv_usec - now.tv_usec + microspersecond * (gps.tv_sec - now.tv_sec);

  if (delta.tv_usec < 2000 && delta.tv_usec > -2000)
  {
    if (showBytes)
    {
      logDebug("Forget about small system clock skew %d\n", delta.tv_usec);
    }
    return;
  }

  if (adjtime(&delta, &olddelta))
  {
    logError("Failed to adjust system clock by %d usec\n", delta.tv_usec);
    return;
  }

  if (showBytes)
  {
    logDebug("Now = %" PRIu64 "/%06u ", (uint64_t) now.tv_sec, now.tv_usec);
    logDebug("GPS = %" PRIu64 "/%06u ", (uint64_t) gps.tv_sec, gps.tv_usec);
    logDebug("Adjusting system clock by %d usec\n", delta.tv_usec);
    if (olddelta.tv_sec || olddelta.tv_usec)
    {
      logDebug("(Old delta not yet completed %" PRIu64 "/%d\n", (uint64_t) olddelta.tv_sec, olddelta.tv_usec);
    }
  }

#endif
#endif
}

void f_ngt1::fillManufacturers(void)
{
  size_t i;

  for (i = 0; i < ARRAY_SIZE(manufacturer); i++) {
    manufacturer[i] = 0;
  }
  for (i = 0; i < ARRAY_SIZE(companyList); i++)
  {
    manufacturer[companyList[i].id] = companyList[i].name;
  }
}

void f_ngt1::fillFieldCounts(void)
{
  size_t i, j;

  for (i = 0; i < pgnListSize; i++)
  {
    for (j = 0; pgnList[i].fieldList[j].name && j < ARRAY_SIZE(pgnList[i].fieldList); j++);
    if (j == ARRAY_SIZE(pgnList[i].fieldList))
    {
      logError("Internal error: PGN %d '%s' does not have correct fieldlist.\n", pgnList[i].pgn, pgnList[i].description);
      exit(2);
    }
    if (j == 0 && pgnList[i].known)
    {
      logError("Internal error: PGN %d '%s' does not have fields.\n", pgnList[i].pgn, pgnList[i].description);
      exit(2);
    }
    pgnList[i].fieldCount = j;
  }
}

void f_ngt1::writeMessage(AWS_SERIAL handle, unsigned char command, const unsigned char * cmd,
			  const size_t len)
{
  unsigned char bst[255];
  unsigned char *b = bst;
  unsigned char *lenPtr;
  unsigned char crc;

  int i;

  *b++ = DLE;
  *b++ = STX;
  *b++ = command;
  crc = command;
  lenPtr = b++;

  for (i = 0; i < len; i++)
  {
    if (cmd[i] == DLE)
    {
      *b++ = DLE;
    }
    *b++ = cmd[i];
    crc += (unsigned char) cmd[i];
  }

  *lenPtr = i;
  crc += i;

  *b++ = (unsigned char) (256 - (int)crc);
  *b++ = DLE;
  *b++ = ETX;

  if (write_serial(handle, (char*)bst, b - bst) != b - bst)
  {
    logError("Unable to write command '%.*s' to NGT-1-A device\n", (int) len, cmd);
  }
  logDebug("Written command %X len %d\n", command, (int) len);
}
