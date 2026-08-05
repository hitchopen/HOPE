#pragma once
#include "libmotioncapture/motioncapture.h"
#include <mutex>
#include <thread>
#include <sstream>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/uuid/uuid.hpp>
#define MAX_PACKET_SIZE 65535
#define MAX_FRAME_SIZE 65535
using namespace boost::asio::ip;
using namespace boost::asio;
using namespace boost;
using namespace std;

typedef unsigned char byte;
typedef byte octet;

typedef char char8;
typedef wchar_t char16;

typedef char int8;
typedef short int16;
typedef int int32;
typedef long int64;

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef unsigned long uint64;

typedef float real32;
typedef double real64;

typedef uint8 uuid[16];

namespace libmotioncapture {
	//get an empty byte buffer of specified size
	inline byte* GetEmptyBuffer(const uint32 uSize) {
		byte* pBuffer = new byte[uSize];
		memset(pBuffer, 0, uSize);
		return pBuffer;
	}

	//clear the specified buffer
	inline void EmptyBuffer(byte* const pBuffer, const uint32 uSize) {
		memset(pBuffer, 0, uSize);
	}

	//copy data from one buffer to the other buffer
	inline void CopyBuffer(byte* const pDst, const byte* const pSrc, const uint32 uSize) {
		memcpy(pDst, pSrc, uSize);
	}

	//release allocated buffer
	inline void ReleaseBuffer(byte* const pBuffer) {
		delete[] pBuffer;
	}
	// Marker
	typedef struct
	{
		uint32 ID;                         // Marker ID:
		struct {
			real32 x;
			real32 y;
			real32 z;
		}sPosition;
	}LMarker;

	// Rigidbody Data
	typedef struct
	{
		uint32 ID;
		struct {
			real32 x;
			real32 y;
			real32 z;
		}sPosition;							//Position
		struct {
			real32 qx;
			real32 qy;
			real32 qz;
			real32 qw;
		}sOrientation;						// Orientation                
		uint32 uTrack;                      // tracking flags
	}LRigidBody;

	// Rigidbody Tag
	typedef struct {
		char8 szName[256];
		uint32 uRigidbodyID;
		struct {
			real32 x;
			real32 y;
			real32 z;
		}sCenteroidTransform;
	}LRigidbodyTag;
	
	// Message
	typedef enum {
		Connect,						//connection request				
		Disconnect,						//disconnection request

		Connected,						//connected status
		Disconnected,					//dosconnected status

		RequestTagList,					//request model definition
		RequestData,					//request motion capture data

		TagListData,					//model definitino
		MotionCaptureData,				//motion capture data

		Ready,							//ready status
		Busy							//busy status
	}Message;

	//constexpr uuid id = { 219, 49, 232, 58, 66, 199, 72, 92, 167, 100, 237, 202, 8, 78, 168, 29 };		//FZMotion UUID:DB31E83A-42C7-485C-A764-EDCA084EA81D
	
	typedef struct {
		Message iMessage;
	}SimpleMessage;
	
	typedef struct {
		//uuid uid;													//uuid - universally unique identifier - 16 bytes
		Message eMessage;											//status of data sending end 
		uint16 uDataBytes;											//bytes of transmitted data - 2 bytes
		char8 szSoftware[256];										//name of sent software - 16 bytes

		union {			
			struct { uint8 v1; uint8 v2; uint8 v3; uint8 v4; };
			uint8 version[4];
		}uVersion;													//software version of sending end - 4 bytes
		
		union {
			struct { uint8 v1; uint8 v2; uint8 v3; uint8 v4; };
			uint8 version[4];
		}uSdkVersion;												//network module version - 4 bytes
		
		uint16 uDataPort;											//data transmission port - 2 bytes
		
		union {
			struct { octet h1; octet h2; octet l1; octet l2; };
			octet ipv4[4];
		}uMulticastGroup;											//ip address of multicast group - 4 bytes
		//uint8 uOptions;											//options for the data transmission - 1 byte
	}SimpleConfirmMessage;	

	class MotionCaptureFZMotion : public MotionCapture {
	private:
		MotionCaptureFZMotion() = delete;
		MotionCaptureFZMotion(const MotionCaptureFZMotion& mcl) = delete;
		MotionCaptureFZMotion& operator=(const MotionCaptureFZMotion& mcl) = delete;

		MotionCaptureFZMotion(const string& strLocalIP, 
			const int iLocalPort, const string& strRemoteIP, const int iRemotePort);

		static MotionCaptureFZMotion* s_pInstance;
		static recursive_mutex s_mutex;

		boost::asio::io_context m_IOContext;
		udp::socket m_TransmissionSocket;
		udp::socket m_ConnectionSocket;
		udp::resolver m_Resolver;

		udp::endpoint m_localCEndpoint;				//local connection endpoint
		udp::endpoint m_remoteCEndpoint;			//remote connection endpoint
		udp::endpoint m_localMEndpoint;				//local multicast endpoint - data transmisson endpoint
		udp::endpoint m_remoteMEndpoint;			//remote multicast endpoint - data transmission endpoint

		int32 m_iLocalCPort;
		int32 m_iRemoteCPort;
		int32 m_iDataReceivePort;

		mutable int32 m_uPreviousFrame;
		int32 m_uFrameNumber;

		uint32 m_uPagkageSize;

		atomic<bool> m_bIsConnected;
		atomic<bool> m_bFirstFrame;

		string m_strLocalIP;
		string m_strRemoteIP;

		string m_strSoftware;
		string m_strSDKVersion;
		string m_strSoftwareVersion;
		string m_strMulticastGroup;

		vector<LMarker> m_vctMarkData;
		vector<LRigidBody> m_vctRigidbodyData;
		mutable map<uint32, LRigidbodyTag> m_mapRigidbodyTagList;
		
		//initailzie the instance
		void init();

		//parse message received form the server
		void parseMessage(const SimpleConfirmMessage& scm);

		//parse rigidbody tag list
		void parseRigidbodyTagList(const byte* const pData, map<uint32, LRigidbodyTag>& mapTagList);

		//parse marker and rigibody data
		void parseData(const byte* const pData, vector<LMarker>& allMarkers, vector<LRigidBody>& allRigidBodys);

		//receive and parse each frame data
		void receiveFrameData();

		//set the connection flag
		inline void setConnected(const bool bIsConnected) { this->m_bIsConnected = bIsConnected; }

		//set the first frame flag
		inline void setFirstFrame(const bool bFirstFrame) { this->m_bFirstFrame = bFirstFrame; }
	protected:
	public:
		//get current unque instance
		inline static MotionCaptureFZMotion* getInstance() {
			return s_pInstance;
		}

		//get unique instance with specified parameter
		static MotionCaptureFZMotion* getInstance(
			const string& strLocalIP, const int iLocalPort, const string& strRemoteIP, const int iRemotePort) {
			s_mutex.lock();

			if (s_pInstance == nullptr) {
				s_pInstance = new MotionCaptureFZMotion(strLocalIP, iLocalPort, strRemoteIP, iRemotePort);
			}
			else {
				s_pInstance->disconnect();
				s_pInstance->setConnectionInfo(strLocalIP, iLocalPort, strRemoteIP, iRemotePort);
				s_pInstance->connect();
			}

			s_mutex.unlock();
			return s_pInstance; 
		}

		virtual ~MotionCaptureFZMotion() {
			s_mutex.lock();

			this->disconnect();

			if (s_pInstance != nullptr)
				delete(s_pInstance);

			s_pInstance = nullptr;

			s_mutex.unlock();
		}
		
		//set both local and remote host ip and port
		void setConnectionInfo(const string& strLocalIP, const int iLocalPort, const string& strRemoteIP, const int iRemotePort);
		
		//connect with the server
		bool connect();
		
		//disconnect with the server and clean all data
		void disconnect();

		inline bool isConnected() const { return this->m_bIsConnected; }

		//overload virtual functions
		void waitForNextFrame();
		const std::map<std::string, RigidBody>& rigidBodies() const;
		const PointCloud& pointCloud() const;
		inline bool supportsRigidBodyTracking() const { return true; }
		inline bool supportsPointCloud() const { return true; }
	};
}
