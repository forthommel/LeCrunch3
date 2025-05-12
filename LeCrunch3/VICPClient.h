//------------------------------------------------------------------------------------------
// Summary:		Lightweight VICP client implementation.
//
// Started by:	Anthony Cake
//
// Started:		June 2003
//				Published on SourceForge under LeCroyVICP project, Sept 2003
//------------------------------------------------------------------------------------------
//
// Description:
//
//		This file contains a Client-side implementation of the VICP network communications
//		protocol used to control LeCroy Digital Oscilloscopes (DSOs).
//
//		This file is intended to be ultimately as platform independent as possible, but at
//		present has only been compiled & tested under Visual C++ 6.0 on a Windows platform.
//
// VICP Protocol Description/History:
//
//		The VICP Protocol has been around since 1997/98. It did not change in any way between it's
//		conception, and June 2003, when a previously reserved field in the header was assigned.
//		This field, found at byte #2, is now used to allow the client-end of a VICP communication
//		to detect 'out of sync' situations, and therefore allows the GPIB 488.2 'Unread Response'
//		mechanism to be emulated.
//
//      These extensions to the original protocol did not cause a version number change, and are
//		referred to as version 1a. It was decided not to bump the version number to reduce the
//		impact on clients, many of which are looking for a version number of 1.
//		Clients and servers detect protocol version 1a by examining the sequence number field,
//		it should be 0 for version 1 of the protocol (early clients), or non-zero for v1a.
//
//
// VICP Headers:
//
//	    Byte	Description
//	    ------------------------------------------------
//		 0		Operation
//		 1		Version		1 = version 1
//		 2		Sequence Number { 1..255 }, (was unused until June 2003)
//		 3		Unused
//	 	 4		Block size, LSB	 (not including this header)
//		 5		Block size
//		 6		Block size
//		 7		Block size, MSB
//
//	Operation bits:
//
//		Bit		Mnemonic	Purpose
//		-----------------------------------------------
//		D7		DATA		Data block (D0 indicates with/without EOI)
//		D6		REMOTE		Remote Mode
//		D5		LOCKOUT		Local Lockout (Lockout front panel)
//		D4		CLEAR		Device Clear (if sent with data, clear occurs before block is passed to parser)
//		D3		SRQ			SRQ (Device -> PC only)
//		D2		SERIALPOLL  Request a serial poll
//		D1		Reserved	Reserved for future expansion
//		D0		EOI			Block terminated in EOI
//
// Known Limitations:
//
// Outstanding Issues
//
// Dependencies
//		- Try to keep to absolute minimum to allow porting to other operating systems.
//		  Avoid ATL, and especially MFC, currently only relies upon winsock.h and the standard
//		  C library includes.
//
//------------------------------------------------------------------------------------------
//

#ifndef LeCrunch3_VICPClient_h
#define LeCrunch3_VICPClient_h

#include <netinet/in.h>
#include <sys/socket.h>

#include <cstdint>
#include <string>

#define ATLTRACE
#define ATLTRACE2

class VICPClient {
public:
  explicit VICPClient(const std::string& device_address);
  virtual ~VICPClient();

  void setTimeout(float);  ///< set the timeout (in s) for all communication parts

  bool openSocket();  ///< initialize the socket (doesn't require remote device to be connected or responding)
  /// connect to a network device
  /// \note address is extracted from device address (specified during construction of base class)
  bool connectToDevice();
  bool disconnectFromDevice();  ///< disconnect from a network device

  void deviceClear();  ///< clear the device
  /// serial poll byte
  /// \note uses the new Out-Of-Band signalling technique if supported, else use the original 'in-band' technique.
  int serialPoll();
  /// out-of band data request, used for serial polling and possibly other features in the future
  bool oobDataRequest(char requestType, unsigned char* response);
  /// send a block of data to a network device
  /// \return false on error status
  bool sendDataToDevice(const std::string& message,
                        bool eoiTermination,
                        bool deviceClear = false,
                        bool serialPoll = false);
  /// dump data until the next header is found
  /// \todo Handle timeout cases
  void dumpData(int numBytes);
  /// read block of data from a network device
  /// \note if bFlush is requested then ignore replyBuf and userBufferSizeBytes and read all remaining data
  /// from the current block (i.e. up to the start of the next header)
  uint32_t readDataFromDevice(char* replyBuf, int userBufferSizeBytes, bool bFlush = false);
  /// read header a network device
  bool readHeaderFromDevice(uint32_t& blockSize, bool& eoiTerminated, bool& srqStateChanged, int& seqNum);

private:
  uint32_t GetDeviceIPAddress();
  uint32_t GetIPFromDNS();  ///< Lookup the IP address of a DNS name
  /// Return the next sequence number in the range 1..255 (Note that zero is omitted intentionally)
  /// used to synchronize write/read operations, attempting to emulate the 488.2 'discard unread response'
  /// behaviour
  unsigned char GetNextSequenceNumber(unsigned char flags);
  /// return the last-used sequence number without incrementing it
  unsigned char GetLastSequenceNumber();
  /// send a 'small' block of data to a network device
  /// \return true on error status
  bool sendSmallDataToDevice(const std::string& message, bool eoiTermination, bool deviceClear, bool serialPoll);

  ::timeval timeout_;
  bool m_remoteMode{false};        ///< if TRUE, device is in remote mode
  bool m_localLockout{false};      ///< if TRUE, device is in local lockout mode
  bool m_connectedToScope{false};  ///< connected to scope?
  sockaddr_in m_serverAddr;        // server's socket address
  int fd_{-1};                     ///< socket file descriptor
  int m_iberr{0};                  ///< emulation of GPIB counterparts
  int m_ibsta{0};                  ///< emulation of GPIB counterparts
  long m_ibcntl{0};                ///< emulation of GPIB counterparts
  int m_maxBlockSize{512};         ///< max # bytes that may be read in one go by recv
  enum readstate {
    NetWaitingForHeader,
    NetWaitingForData,
    NetErrorState
  } m_readState{readstate::NetWaitingForHeader};  ///< current state of read 'state machine'
  bool m_bFlushUnreadResponses{true};     ///< if true, unread responses are flushed (emulate GPIB 488.2 behaviour)
  bool m_bErrorFlag{false};               ///< if true, error has been observed
  bool m_bVICPVersion1aSupported{false};  ///< version 1a of the VICP protocol supported (seq. numbers and OOB data)

  static constexpr unsigned short SERVER_PORT_NUM = 1861;  ///< port # registered with IANA for lecroy-vicp
  static constexpr size_t IO_NET_HEADER_SIZE = 8;          ///< size of network header
  static constexpr size_t SMALL_DATA_BUFSIZE = 8192;
  static constexpr unsigned long CONNECT_TIMEOUT_SECS = 2;
  static constexpr size_t SPOLLBUFSIZE = 2;

  const std::string device_address_;

  int m_lastSequenceNumber{1};  ///< last used sequence value
  int m_nextSequenceNumber{1};  ///< next sequence value
};

#endif
