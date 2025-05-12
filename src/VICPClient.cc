//------------------------------------------------------------------------------------------
// Summary:		Lightweight VICP client implementation.
//
// Started by:	Anthony Cake
//
// Started:		June 2003
//				Published on SourceForge under LeCroyVICP project, Sept 2003
//------------------------------------------------------------------------------------------

#include <arpa/inet.h>
#include <ctype.h>
#include <malloc.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/param.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <vector>

#include "LeCrunch3/VICPClient.h"

using namespace std::string_literals;

/// GPIB status bit vector (global variable ibsta and wait mask)
enum ibsta_t : uint16_t {
  ERR = (1 << 15),    ///< Error detected
  TIMO = (1 << 14),   ///< Timeout
  END = (1 << 13),    ///< EOI or EOS detected
  SRQI = (1 << 12),   ///< SRQ detected by CIC
  RQS = (1 << 11),    ///< Device needs service
  SPOLL = (1 << 10),  ///< Board has been serially polled
  CMPL = (1 << 8),    ///< I/O completed
  REM = (1 << 6),     ///< Remote state
  CIC = (1 << 5),     ///< Controller-in-Charge
  ATN = (1 << 4),     ///< Attention asserted
  TACS = (1 << 3),    ///< Talker active
  LACS = (1 << 2),    ///< Listener active
  DTAS = (1 << 1),    ///< Device trigger state
  DCAS = (1 << 0)     ///< Device clear state
};

/// GPIB error codes
enum iberr_t : uint8_t {
  EDVR = 0,   ///< System error
  ECIC = 1,   ///< Function requires GPIB board to be CIC
  ENOL = 2,   ///< Write function detected no Listeners
  EADR = 3,   ///< Interface board not addressed correctly
  EARG = 4,   ///< Invalid argument to function call
  ESAC = 5,   ///< Function requires GPIB board to be SAC
  EABO = 6,   ///< I/O operation aborted
  ENEB = 7,   ///< Non-existent interface board
  EDMA = 8,   ///< Error performing DMA
  EOIP = 10,  ///< I/O operation started before previous operation completed
  ECAP = 11,  ///< No capability for intended operation
  EFSO = 12,  ///< File system operation error
  EBUS = 14,  ///< Command error during device call
  ESTB = 15,  ///< Serial poll status byte lost
  ESRQ = 16,  ///< SRQ remains asserted
  ETAB = 20,  ///< The return buffer is full.
  ELCK = 21   ///< Address or board is locked.
};

/// VICP header 'Operation' bits
enum VICPOperation : uint8_t {
  OPERATION_DATA = 0x80,
  OPERATION_REMOTE = 0x40,
  OPERATION_LOCKOUT = 0x20,
  OPERATION_CLEAR = 0x10,
  OPERATION_SRQ = 0x08,
  OPERATION_REQSERIALPOLL = 0x04,
  OPERATION_EOI = 0x01
};

/// Header Version
enum HeaderVersion { HEADER_VERSION1 = 0x01 };

VICPClient::VICPClient(const std::string& device_address) : device_address_(device_address) {}

VICPClient::~VICPClient() { disconnectFromDevice(); }

uint32_t VICPClient::GetDeviceIPAddress() {
  // loop through the address string and try to identify whether it's a dotted static IP
  // address, or a DNS address
  bool bOnlyDigitsAndDots = true;
  size_t dotCount = 0;
  for (size_t i = 0; i < device_address_.size(); ++i) {
    if (device_address_[i] == '.')  // count the dots
      ++dotCount;
    if (!isdigit(device_address_[i]) &&
        device_address_[i] != '.')  // if the character is not a digit and not a dot then assume a DNS name
      bOnlyDigitsAndDots = false;
  }

  if (bOnlyDigitsAndDots &&
      dotCount == 3) {  // if only digits and dots were found then assume that it's a static IP address
    in_addr ipAddr;
    int b1 = 0, b2 = 0, b3 = 0, b4 = 0;
    if (int ret = ::sscanf(device_address_.data(), "%d.%d.%d.%d", &b1, &b2, &b3, &b4); ret != 4) {
      std::cerr << "GetDeviceIPAddress() sscanf error ret=" << ret << "/4" << std::endl;
      m_bErrorFlag = true;
    }
    ::inet_pton(AF_INET, device_address_.data(), &ipAddr.s_addr);
    return ipAddr.s_addr;
  }

  // assume that it's DNS name
  return GetIPFromDNS();
}

uint32_t VICPClient::GetIPFromDNS() {
  struct in_addr addr;
  int ret, *intp;
  char** p;
  struct hostent* hp = ::gethostbyname(device_address_.data());
  if (hp != nullptr) {
    // We assume that if there is more than 1 address, there might be
    // a server conflict.
    p = hp->h_addr_list;
    intp = (int*)*p;
    ret = *intp;
    addr.s_addr = ret;
    return addr.s_addr;
  }

  return 0;
}

void VICPClient::setTimeout(float timeout) {
  timeout_ = ::timeval{(long)timeout, ((long)(timeout * 1'000'000L)) % 1'000'000L};
}

bool VICPClient::openSocket() {
  ATLTRACE("Opening Socket:\n");

  // create client's socket
  if (fd_ = socket(AF_INET, SOCK_STREAM, 0); fd_ == -1) {
    std::cerr << "socket() failed, error code = " << errno << std::endl;
    m_bErrorFlag = true;
    // 10061 = WSAECONNREFUSED
    return false;
  }

  // disable the TCP/IP 'NAGLE' algorithm that buffers a bunch of
  // packets before sending them.
  const int just_say_no = 1;
  if (0 != setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, (char*)&just_say_no, sizeof(just_say_no))) {
    printf("Error: Could not set socket option 'TCP_NODELAY'\n");
    return false;
  }

  // enable SO_LINGER to allow hard closure of sockets (LINGER enabled, but with timeout of zero)
  linger lingerStruct = {1, 0};
  if (0 != setsockopt(fd_, SOL_SOCKET, SO_LINGER, (char*)&lingerStruct, sizeof(lingerStruct))) {
    printf("Error: Could not set socket option 'SO_LINGER'\n");
    return false;
  }

  return true;
}

bool VICPClient::connectToDevice() {
  if (!m_connectedToScope) {  // if not yet connected to scope...
    ATLTRACE("Connecting:\n");
    uint32_t addr = GetDeviceIPAddress();  // lookup the IP address of the device
    openSocket();                          // initialize the socket

    // ensure that the connect command will not block
    /*unsigned long argp = 1;  // 1 = enable non-blocking behaviour
    ioctlsocket(fd_, FIONBIO, &argp);*/

    // try to connect to server (scope)
    const int sockAddrSize = sizeof(sockaddr);  // size of socket address structures

    // build server socket address
    m_serverAddr.sin_family = AF_INET;
    m_serverAddr.sin_port = htons(SERVER_PORT_NUM);
    if ((m_serverAddr.sin_addr.s_addr = addr) == -1) {
      std::cerr << "Bad server address" << std::endl;
      m_bErrorFlag = true;
      m_ibsta = ERR;
      m_iberr = EARG;  // Invalid argument to function call
      m_ibcntl = 0;
      return false;
    }

    int connectStatus = ::connect(fd_, (sockaddr*)&m_serverAddr, sockAddrSize);

    // after a connect in non-blocking mode a 'select' is require to
    // determine the outcome of the connect attempt
    ::fd_set writeSet = {1, {fd_}};
    ::timeval timeVal{CONNECT_TIMEOUT_SECS, 0L};
    int numReady = ::select(fd_, nullptr, &writeSet, nullptr, &timeVal);

    // restore blocking behaviour
    /*argp = 0;  // 0 = enable blocking behaviour
    ioctlsocket(fd_, FIONBIO, &argp);*/

    if (numReady == 1) {  // see if the connection succeeded
      ATLTRACE("Connect Succeeded\n");
      m_connectedToScope = true;
    } else {
      std::cerr << "socket() failed, error code = " + errno << std::endl;
      m_bErrorFlag = true;
      disconnectFromDevice();
      m_ibsta = ERR;
      m_iberr = EABO;  // I/O operation aborted
      m_ibcntl = 0;
      return false;
    }
  }
  return true;
}

bool VICPClient::disconnectFromDevice() {
  ATLTRACE("Disconnecting:\n");
  if (m_connectedToScope) {
    m_readState = NetWaitingForHeader;  // reset any partial read operation
    if (::close(fd_) != 0) {
      std::cerr << "close() failed, error code = " << errno << std::endl;
      m_bErrorFlag = true;
    }
    fd_ = -1;
    m_connectedToScope = false;
    m_bVICPVersion1aSupported = false;
  }
  return true;
}

void VICPClient::deviceClear() {
  sendDataToDevice("", 0, 0, true /* device clear */);
  usleep(100'000);  // TODO: remove when 'RebootScope' bug is fixed
  disconnectFromDevice();
  connectToDevice();
}

int VICPClient::serialPoll() {
  if (m_bVICPVersion1aSupported) {
    unsigned char oobResponse = 0x00;
    if (oobDataRequest('S', &oobResponse))  // 'S' == Serial Poll
      return oobResponse;
    m_ibsta = ERR;
    m_iberr = EABO;  // The serial poll response could not be read within the serial poll timeout period.
    std::cerr << "serialPoll failed to receive OOB response from DSO" << std::endl;
    m_bErrorFlag = true;
    return oobResponse;
  }
  // request the serial poll using an in-band technique
  sendDataToDevice(""s, false /* EOI */, false /* device clear */, true /* request serial poll */);

  char buf[SPOLLBUFSIZE + 1];
  if (int bytesRead = readDataFromDevice(buf, SPOLLBUFSIZE); bytesRead >= 1) {  // read the single serial-poll byte
    m_ibsta = CMPL;
    return buf[0];
  }
  m_ibsta = ERR;
  m_iberr = EABO;  // The serial poll response could not be read within the serial poll timeout period.
  return 0;
}

bool VICPClient::oobDataRequest(char requestType, unsigned char* response) {
  char oobDataTest[1] = {requestType};
  int bytesSent = ::send(fd_, (char*)oobDataTest, 1, MSG_OOB);

  // if there is no sign of a header, get out quick (timeout)
  // Note that we don't look for in-band data, only OOB data (which appears in the exception record)
  ::fd_set exceptSet = {1, {fd_}};
  int numReady = ::select(fd_, NULL, NULL, &exceptSet, &timeout_);
  char buf[1] = {0x00};
  if (FD_ISSET(fd_, &exceptSet)) {
    int bytesReceived = ::recv(fd_, (char*)buf, 1, MSG_OOB);
    *response = buf[0];
    return true;
  }
  return false;
}

bool VICPClient::sendDataToDevice(const std::string& message, bool eoiTermination, bool deviceClear, bool serialPoll) {
  if (m_bFlushUnreadResponses &&
      m_readState != NetWaitingForHeader)  // handle cases where the user didn't read all data from a previous query
    readDataFromDevice(nullptr, -1, true);    // flush

  if (message.size() <
      SMALL_DATA_BUFSIZE)  // if the block is relatively small, send header + data with one 'send' command (faster)
    return sendSmallDataToDevice(message, eoiTermination, deviceClear, serialPoll);

  m_ibsta &= RQS;  // preserve SRQ
  m_ibcntl = 0;      // clear status words
  m_iberr = 0;

  std::array<unsigned char, IO_NET_HEADER_SIZE> headerBuf;  // send header
  headerBuf[0] = OPERATION_DATA;
  if (eoiTermination)
    headerBuf[0] |= OPERATION_EOI;
  if (m_remoteMode)
    headerBuf[0] |= OPERATION_REMOTE;
  if (deviceClear)
    headerBuf[0] |= OPERATION_CLEAR;
  if (serialPoll)
    headerBuf[0] |= OPERATION_REQSERIALPOLL;
  headerBuf[1] = HEADER_VERSION1;
  headerBuf[2] = GetNextSequenceNumber(headerBuf[0]);          // sequence number
  headerBuf[3] = 0x00;                                         // unused
  *reinterpret_cast<unsigned long*>(&headerBuf[4]) = ::htonl(message.size());  // message size

  ATLTRACE("sendDataToDevice: seq=%d eoi=%d ", headerBuf[2], eoiTermination);

  int bytesSent = 0;
  if (bytesSent = ::send(fd_, headerBuf.data(), headerBuf.size(), 0); bytesSent != IO_NET_HEADER_SIZE) {
    std::cerr << "Could not send Net Header" << std::endl;
    m_bErrorFlag = true;
    m_ibsta |= ERR;
    ATLTRACE(" bytesSent=%d [%.20s] ERROR sending header\n", bytesSent, message);
    return false;
  }
  if (bytesSent = ::send(fd_, message.data(), message.size(), 0);
      bytesSent == -1 || bytesSent != message.size()) {  // send contents of message
    std::cerr << "Send error" << std::endl;
    m_bErrorFlag = true;
    m_ibsta |= ERR;
    ATLTRACE(" bytesSent=%d [%.20s] ERROR sending data\n", bytesSent, message);
    return false;
  }
  ATLTRACE(" bytesSent=%d [%.20s]\n", bytesSent, message);
  m_ibsta = CMPL | CIC | TACS;
  m_ibcntl = bytesSent;
  return true;
}

unsigned char VICPClient::GetNextSequenceNumber(unsigned char flags) {
  m_lastSequenceNumber = m_nextSequenceNumber;  // we'll return the current sequence number
  if (flags & OPERATION_EOI) {                  // which then gets incremented if this block is EOI terminated
    ++m_nextSequenceNumber;
    if (m_nextSequenceNumber >= 256)
      m_nextSequenceNumber = 1;
  }
  return m_lastSequenceNumber;
}

unsigned char VICPClient::GetLastSequenceNumber() { return m_lastSequenceNumber; }

bool VICPClient::sendSmallDataToDevice(const std::string& message,
                                       bool eoiTermination,
                                       bool deviceClear,
                                       bool serialPoll) {
  std::array<unsigned char, SMALL_DATA_BUFSIZE + IO_NET_HEADER_SIZE + 2> smallDataBuffer{};
  const size_t bytesToSendWithHeader = message.size() + IO_NET_HEADER_SIZE;

  memcpy(&smallDataBuffer[IO_NET_HEADER_SIZE], message.data(), message.size());  // copy message into data buffer

  m_ibsta &= (RQS);  // preserve SRQ
  m_ibcntl = 0;      // clear status words
  m_iberr = 0;

  smallDataBuffer[0] = OPERATION_DATA;  // send header + data
  if (eoiTermination)
    smallDataBuffer[0] |= OPERATION_EOI;
  if (m_remoteMode)
    smallDataBuffer[0] |= OPERATION_REMOTE;
  if (deviceClear)
    smallDataBuffer[0] |= OPERATION_CLEAR;
  if (serialPoll)
    smallDataBuffer[0] |= OPERATION_REQSERIALPOLL;
  smallDataBuffer[1] = HEADER_VERSION1;
  smallDataBuffer[2] = GetNextSequenceNumber(smallDataBuffer[0]);    // sequence number
  smallDataBuffer[3] = 0x00;                                         // unused
  *((unsigned long*)&smallDataBuffer[4]) = ::htonl(message.size());  // message size

  ATLTRACE("sendSmallDataToDevice: seq=%d eoi=%d ", smallDataBuffer[2], eoiTermination);

  if (int bytesSent = send(fd_, smallDataBuffer.data(), bytesToSendWithHeader, 0);
      bytesSent != bytesToSendWithHeader) {
    std::cerr << "Could not send small data block (Header + Data)" << std::endl;
    m_bErrorFlag = true;
    m_ibsta |= ERR;
    ATLTRACE(" bytesSent=%d [%.20s] ERROR\n", bytesSent, message);
    return false;
  } else {
    ATLTRACE(" bytesSent=%d [%.20s]\n", bytesSent, message);
    m_ibsta = CMPL | CIC | TACS;
    m_ibcntl = message.size();
    return true;
  }
}

void VICPClient::dumpData(int numBytes) {
  std::cerr << "dumpData: Unread Response, dumping " << numBytes << " bytes" << std::endl;
  uint32_t bytesToGo = numBytes;
  if (m_maxBlockSize == 0)
    return;
  std::vector<char> dumpBuf(m_maxBlockSize);
  while (bytesToGo > 0) {
    const int dumpBytesReceived =
        static_cast<uint32_t>(::recv(fd_, dumpBuf.data(), MIN(bytesToGo, (uint32_t)m_maxBlockSize), 0));
    bytesToGo -= dumpBytesReceived;
  }
}

uint32_t VICPClient::readDataFromDevice(char* replyBuf, int userBufferSizeBytes, bool bFlush) {
  static uint32_t srcBlockSizeBytes = 0, srcBlockBytesRead = 0;
  static bool srcBlockEOITerminated = false, srcBlockSRQStateChanged = false;
  int userBufferBytesRead = 0;  // # bytes placed in user buffer
  uint32_t bytesReceived = 0;
  char* bufCopy = replyBuf;

  m_ibsta &= (RQS);        // preserve SRQ
  m_ibcntl = m_iberr = 0;  // clear status words

  if (replyBuf != nullptr)  // ensure that the reply buffer is empty (if supplied)
    *replyBuf = '\0';

  while (true) {
    if (m_readState == NetWaitingForHeader) {
      if (int seqNum = -1; readHeaderFromDevice(srcBlockSizeBytes,
                               srcBlockEOITerminated,
                               srcBlockSRQStateChanged,
                               seqNum)) {  // the header was successfully read
        ATLTRACE("readDataFromDevice: Read Header: blockSize=%d, EOI=%d, userBufferSizeBytes=%d\n",
                 srcBlockSizeBytes,
                 srcBlockEOITerminated,
                 userBufferSizeBytes);
        m_readState = NetWaitingForData;
        srcBlockBytesRead = 0;

        // if we are flushing unread responses, and this header contains an unexpected sequence number (older than
        // the current one), then dump this block and go around again.
        // note that the 'seqNum != 0' test checks for the case where we are talking to a scope running pre-June 2003
        // code that did not support sequence numbering, and therefore we do not know when to dump data.
        if (m_bFlushUnreadResponses && seqNum != 0 && (GetLastSequenceNumber() > seqNum)) {
          dumpData(srcBlockSizeBytes);
          m_readState = NetWaitingForHeader;
        }

        // if a non-zero sequence number was seen, then assume that version '1a' of the
        // VICP protocol is in use, supporting, in addition to sequence numbering,
        // the use of Out-of-band signalling.
        m_bVICPVersion1aSupported = (seqNum != 0);  // seq. numbers should never be zero in V1a of the protocol
      } else {                                      // header was not successfully read, probably indicates a timeout
        //ATLTRACE("readDataFromDevice Timeout reading header\n");
        m_ibsta |= ERR;
        m_ibsta |= TIMO;  // let the caller know that a timeout occured
        break;
      }
    }

    if (m_readState == NetWaitingForData) {
      if (bFlush) {  // dump any unread partial buffer if requested
        dumpData(srcBlockSizeBytes - srcBlockBytesRead);
        m_readState = NetWaitingForHeader;
        break;
      }

      // fill the user-supplied buffer
      uint32_t bytesToGo = MIN((uint32_t)userBufferSizeBytes - userBufferBytesRead,  // space free in user Buffer
                               srcBlockSizeBytes - srcBlockBytesRead);               // src bytes available
      while (bytesToGo > 0) {
        if (bytesReceived = static_cast<uint32_t>(::recv(fd_, replyBuf, MIN(bytesToGo, (uint32_t)m_maxBlockSize), 0));
            bytesReceived == -1) {
          std::cerr << "SOCKET_ERROR on recv" << std::endl;
          m_bErrorFlag = true;
          m_readState = NetErrorState;
          m_ibsta |= ERR;
          m_ibsta |= TIMO;  // let the caller know that a timeout occured
          break;
        } else if (bytesReceived > 0) {
          userBufferBytesRead += bytesReceived;
          bytesToGo -= bytesReceived;
          replyBuf += bytesReceived;
          srcBlockBytesRead += bytesReceived;
        }
      }

      // if we have finished reading the contents of this header-prefixed block,
      // then go back to the state where we can watch for the next block
      if (srcBlockBytesRead >= srcBlockSizeBytes) {
        m_readState = NetWaitingForHeader;

        if (srcBlockSRQStateChanged) {  // update SRQ status bits, discard SRQ packet
          if (bufCopy[0] == '1')        // 1 = SRQ asserted, 0 = SRQ deasserted
            m_ibsta |= RQS;
          else
            m_ibsta &= ~(RQS);  // clear SRQ

          // discard SRQ data in buffer
          userBufferBytesRead -= bytesReceived;
          replyBuf -= bytesReceived;
          srcBlockBytesRead -= bytesReceived;

          ATLTRACE("SRQ Packet Discarded  '%c'\n", bufCopy[0]);
          continue;  // go around the loop again (discard SRQ packet)
        }
        if (srcBlockEOITerminated) {  // go around the loop again unless the last block was EOI terminated
          m_ibsta |= END;
          break;
        }
      }
      if (userBufferBytesRead >= userBufferSizeBytes)  // if there is space left in the user's buffer, go around again
        break;
    }
    if (m_readState == NetErrorState) {  // when we come back in here, enter the 'waiting for header' state
      m_readState = NetWaitingForHeader;
      break;
    }
  }
  m_ibcntl = userBufferBytesRead;  // keep track of size of last transfer
  //ATLTRACE("readDataFromDevice: returning %d bytes\n", userBufferBytesRead);
  return m_ibcntl;
}

bool VICPClient::readHeaderFromDevice(uint32_t& blockSize, bool& eoiTerminated, bool& srqStateChanged, int& seqNum) {
  std::array<unsigned char, IO_NET_HEADER_SIZE> headerBuf{};
  ::fd_set readSet = {1, {fd_}};
  if (int numReady = ::select(fd_, &readSet, NULL, NULL, &timeout_);
      numReady != 1)  // if there is no sign of a header, get out quick (timeout)
    return false;
  size_t headerBytesRead = 0;
  while (headerBytesRead < IO_NET_HEADER_SIZE) {  // wait until 8 bytes are available (the entire header)
    if (const auto numReady = ::select(fd_, &readSet, NULL, NULL, &timeout_);
        numReady != 1)  // ensure that the recv command will not block
      break;
    if (const auto bytesThisTime = ::recv(fd_,
                                          reinterpret_cast<char*>(headerBuf.data()) + headerBytesRead,
                                          headerBuf.size() - headerBytesRead,
                                          0);
        bytesThisTime > 0)  // try to read the remainder of the header
      headerBytesRead += bytesThisTime;
    if (headerBytesRead == 0)  // if we get this far without reading any part of the header, get out
      break;
  }
  if (headerBytesRead == IO_NET_HEADER_SIZE) {              // receive the scope's response, header first
    blockSize = ::ntohl(
        *reinterpret_cast<unsigned long*>(&headerBuf[4]));  // extract the number of bytes contained in this packet
    if (!(headerBuf[0] & OPERATION_DATA) || headerBuf[1] != HEADER_VERSION1) {  // check the integrity of the header
      std::cerr << "Invalid Header!" << std::endl;
      m_bErrorFlag = true;
      disconnectFromDevice();  // error state, cannot recognise header. since we are
      connectToDevice();       // out of sync, need to close & reopen the socket
      return false;
    }
    eoiTerminated = headerBuf[0] & OPERATION_EOI;  // inform the caller of the EOI and SRQ state
    srqStateChanged = headerBuf[0] & OPERATION_SRQ;
    seqNum = headerBuf[2];  // inform the caller of the received sequence number
    ATLTRACE("readHeaderFromDevice: seq=%d\n", headerBuf[2]);
    return true;
  }
  disconnectFromDevice();  // error state, cannot read header. since we are out of sync,
  connectToDevice();       // need to close & reopen the socket
  return false;
}
