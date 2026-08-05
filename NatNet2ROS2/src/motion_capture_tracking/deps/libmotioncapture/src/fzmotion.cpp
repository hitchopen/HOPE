#include "libmotioncapture/fzmotion.h"
namespace libmotioncapture {
    recursive_mutex MotionCaptureFZMotion::s_mutex;
    MotionCaptureFZMotion* MotionCaptureFZMotion::s_pInstance = nullptr;
    MotionCaptureFZMotion::MotionCaptureFZMotion(
        const string& strLocalIP, 
        const int iLocalPort, 
        const string& strRemoteIP, 
        const int iRemotePort
    ) : m_ConnectionSocket(m_IOContext),
        m_TransmissionSocket(m_IOContext), 
        m_Resolver(m_IOContext){
        this->init();
        this->setConnectionInfo(strLocalIP, iLocalPort, strRemoteIP, iRemotePort);
        this->connect();
    }
    //initailzie the instance
    void MotionCaptureFZMotion::init() {
        this->m_mapRigidbodyTagList.clear();
        this->m_vctMarkData.clear();
        this->m_vctRigidbodyData.clear();

        this->setConnected(false);
        this->setFirstFrame(true);

        this->m_strLocalIP = "";
        this->m_strRemoteIP = "";

        this->m_strSoftware = "";
        this->m_strSDKVersion = "";
        this->m_strSoftwareVersion = "";
        this->m_strMulticastGroup = "";

        this->m_iLocalCPort = 0;
        this->m_iRemoteCPort = 0;
        this->m_iDataReceivePort = 0;
        this->m_uPagkageSize = 0;
        this->m_uPreviousFrame = -1;
        this->m_uFrameNumber = 0;

        this->m_localCEndpoint = udp::endpoint();			//local connection endpoint
        this->m_remoteCEndpoint = udp::endpoint();			//remote connection endpoint
        this->m_localMEndpoint = udp::endpoint();			//local multicast endpoint - data transmisson endpoint
        this->m_remoteMEndpoint = udp::endpoint();          //remote multicast endpoint - data transmisson endpoint
    }
    //set both local and remote host ip and port
    void MotionCaptureFZMotion::setConnectionInfo(const string& strLocalIP, const int iLocalPort, const string& strRemoteIP, const int iRemotePort) {
        s_mutex.lock();

        this->disconnect();

        this->m_strLocalIP = strLocalIP;
        this->m_iLocalCPort = iLocalPort;
        this->m_strRemoteIP = strRemoteIP;
        this->m_iRemoteCPort = iRemotePort;
        
        this->m_localCEndpoint = udp::endpoint(make_address_v4(this->m_strLocalIP), this->m_iLocalCPort);
        this->m_remoteCEndpoint = udp::endpoint(make_address(this->m_strRemoteIP), this->m_iRemoteCPort);
        
        s_mutex.unlock();
    }
    //connect with the server
    bool MotionCaptureFZMotion::connect() {
        s_mutex.lock();

        if (this->m_ConnectionSocket.is_open() == false) {
            this->m_ConnectionSocket.open(udp::v4());
            this->m_ConnectionSocket.set_option(udp::socket::reuse_address(true));
        }

        SimpleConfirmMessage scm;
        udp::endpoint remoteEndpoint;
        boost::system::error_code ec;
        SimpleMessage sm = { Message::Connect };

        cout << "Connecting..." << endl;

        //create connection request thread
        bool bReceivedConfirmMessage = false;
        thread requestConnection([&]{
        	do{
				//connect to data sending end
				if(this->m_ConnectionSocket.send_to(boost::asio::buffer(&sm, sizeof(sm)), this->m_remoteCEndpoint, 0, ec) <= 0){
					cout << "Sending connection request failed. Error code: " << ec.value() << endl;
					continue;
				}
				cout << "Waiting for response..." << endl;
#ifdef _WIN32
                Sleep(1000);
#else
				sleep(1);
#endif
        	}while(bReceivedConfirmMessage == false);
        });

        while(true){
			//receive confirmation data from the data sending end
			if (this->m_ConnectionSocket.receive_from(boost::asio::buffer(&scm, sizeof(scm)), remoteEndpoint, 0, ec) <= 0) {
				cout << "Failed to receive confirm message. Error code: " << ec.value() << endl;
				continue;
			}

			if (scm.eMessage != Message::Ready) {
				cout << "Data sending host is not ready or receved package is not confirm message." << endl;
				continue;
			}
			bReceivedConfirmMessage = true;
			requestConnection.detach();
			break;
        }

        this->m_remoteCEndpoint = remoteEndpoint;

        //parse received data
        parseMessage(scm);

        //create tag list request thread
        bool bTagListReceived = false;
        sm = { Message::RequestTagList };
        byte* pBuffer = GetEmptyBuffer(MAX_PACKET_SIZE);

        thread requestTagList([&]{
        	do{
                //request rigidbody tag list
                if(this->m_ConnectionSocket.send_to(boost::asio::buffer(&sm, sizeof(sm)), this->m_remoteCEndpoint, 0, ec) <= 0){
                	cout << "Failed to send tag list request. Error code: " << ec.value() << endl;
                	continue;
                };
                cout << "Waiting for response..." << endl;
#ifdef _WIN32
                Sleep(1000);
#else
                sleep(1);
#endif
            }while(bTagListReceived == false);
        });

        while(true){
			//receive rigidbody tag list
			if (this->m_ConnectionSocket.receive_from(boost::asio::buffer(pBuffer, MAX_PACKET_SIZE), this->m_remoteCEndpoint, 0, ec) <= 0) {
				cout << "Failed to receive tag list package. Error code: " << ec.value() << endl;
				continue;
			}

			Message eMessage;
			CopyBuffer((byte*)&eMessage, pBuffer, sizeof(eMessage));

			if (eMessage != Message::TagListData) {
				cout << "Receved package is not tag list." << endl;
				continue;
			}
            //parse received data
            parseRigidbodyTagList(pBuffer, this->m_mapRigidbodyTagList);
	        bTagListReceived = true;
	        requestTagList.detach();
	        EmptyBuffer(pBuffer, MAX_PACKET_SIZE);
			break;
        };

        ReleaseBuffer(pBuffer);

        //joint multicast group
        this->m_localMEndpoint = udp::endpoint(make_address_v4(this->m_strMulticastGroup), this->m_iDataReceivePort);
        if (this->m_TransmissionSocket.is_open() == false) {
            this->m_TransmissionSocket.open(this->m_localMEndpoint.protocol());
            this->m_TransmissionSocket.set_option(udp::socket::reuse_address(true));
            this->m_TransmissionSocket.set_option(ip::multicast::hops(5));
            this->m_TransmissionSocket.set_option(ip::multicast::enable_loopback(true));
            this->m_TransmissionSocket.set_option(ip::multicast::join_group(make_address_v4(this->m_strMulticastGroup), make_address_v4(this->m_strLocalIP)));
            this->m_TransmissionSocket.bind(this->m_localMEndpoint);
        }

        this->setConnected(true);
        cout << "Connected successfully." << endl;
        s_mutex.unlock();
        return true;
    }
    //disconnect with the server and clean all data
    void MotionCaptureFZMotion::disconnect() {
        s_mutex.lock();
        
        this->setConnected(false);
        this->setFirstFrame(true);

        if (this->m_ConnectionSocket.is_open() == true) {
            this->m_ConnectionSocket.close();
        }

        if (this->m_TransmissionSocket.is_open() == true) {
            this->m_TransmissionSocket.set_option(ip::multicast::leave_group(make_address_v4(this->m_strMulticastGroup), make_address_v4(this->m_strLocalIP)));
            this->m_TransmissionSocket.close();
        }

        this->m_mapRigidbodyTagList.clear();
        this->m_vctMarkData.clear();
        this->m_vctRigidbodyData.clear();

        this->m_strLocalIP = "";
        this->m_strRemoteIP = "";

        this->m_strSoftware = "";
        this->m_strSDKVersion = "";
        this->m_strSoftwareVersion = "";
        this->m_strMulticastGroup = "";

        this->m_iLocalCPort = 0;
        this->m_iRemoteCPort = 0;
        this->m_iDataReceivePort = 0;
        this->m_uPagkageSize = 0;
        this->m_uPreviousFrame = -1;
        this->m_uFrameNumber = 0;

        this->m_localCEndpoint = udp::endpoint();			//local connection endpoint
        this->m_remoteCEndpoint = udp::endpoint();			//remote connection endpoint
        this->m_localMEndpoint = udp::endpoint();			//local multicast endpoint - data transmisson endpoint
        this->m_remoteMEndpoint = udp::endpoint();          //remote multicast endpoint - data transmisson endpoint

        s_mutex.unlock();
    }
    //parse message received form the server
    void MotionCaptureFZMotion::parseMessage(const SimpleConfirmMessage& scm) {
        //software name
        this->m_strSoftware = scm.szSoftware;
 
        //buffer size
        this->m_uPagkageSize = scm.uDataBytes;

        //data receive port
        this->m_iDataReceivePort = scm.uDataPort;

        //parse sdk version
        stringstream sstream;
        sstream << (uint32)scm.uSdkVersion.version[0] << "." << (uint32)scm.uSdkVersion.version[1] << 
            "." << (uint32)scm.uSdkVersion.version[2] << "." << (uint32)scm.uSdkVersion.version[3];
        
        this->m_strSDKVersion = sstream.str();

        //parse sdk version
        sstream.str("");
        sstream << (uint32)scm.uVersion.version[0] << "." << (uint32)scm.uVersion.version[1] <<
            "." << (uint32)scm.uVersion.version[2] << "." << (uint32)scm.uVersion.version[3];

        this->m_strSoftwareVersion = sstream.str();

        //parse multicast ip group
        sstream.str(this->m_strMulticastGroup);
        sstream << (uint32)scm.uMulticastGroup.ipv4[0] << "." << (uint32)scm.uMulticastGroup.ipv4[1] << "." << 
            (uint32)scm.uMulticastGroup.ipv4[2] << "." << (uint32)scm.uMulticastGroup.ipv4[3];

        this->m_strMulticastGroup = sstream.str();
    }
    //parse rigidbody tag list
    void MotionCaptureFZMotion::parseRigidbodyTagList(const byte* const pData, map<uint32, LRigidbodyTag>& mapTagList) {
        //parse rigidboy tag list
        byte* ptr = const_cast<byte*>(pData);

        //get message
        Message eMessage;
        CopyBuffer((byte*)&eMessage, ptr, sizeof(Message));

        //validate message
        if (eMessage != Message::TagListData)
            return;

        ptr += sizeof(Message);

        //get byte count of sent data
        uint16 uDataBytes;
        CopyBuffer((byte*)&uDataBytes, ptr, sizeof(uint16));
        this->m_uPagkageSize = uDataBytes;
        ptr += sizeof(uint16);
        
        //get set number
        int32 iDataSetNumber = int32(*ptr);
        CopyBuffer((byte*)&iDataSetNumber, ptr, sizeof(int32));
        ptr += sizeof(int32);

        //get tag list data
        mapTagList.clear();
        for (int i = 0; i < iDataSetNumber; i++) {
            LRigidbodyTag tag;
            strcpy(tag.szName, (char*)ptr);
            int iOffset = static_cast<int>(strlen(tag.szName) + 1);
            ptr += iOffset;
            CopyBuffer((byte*)&tag + 256, ptr, sizeof(uint32) + sizeof(real32) * 3);
            mapTagList[tag.uRigidbodyID] = tag;
            ptr += sizeof(uint32) + sizeof(real32) * 3;
        }
    }
    //receive and parse each frame data
    void MotionCaptureFZMotion::receiveFrameData() {
        byte* pBuffer = GetEmptyBuffer(MAX_FRAME_SIZE);
        boost:system::error_code ec;
        udp::endpoint multicastEndpoint;
        cout << "receiveiFrameData" << endl;
        do{
        	//receive motion capture data
            if (this->m_TransmissionSocket.receive_from(boost::asio::buffer(pBuffer, MAX_FRAME_SIZE), multicastEndpoint,0, ec) <= 0) {
                cout << "Failed to receve data frame." << endl;
            	continue;
            }
            //get message
            Message eMessage;
            CopyBuffer((byte*)&eMessage, pBuffer, sizeof(Message));

            if (eMessage != Message::MotionCaptureData) {
                continue;
            }

            if (this->m_bFirstFrame == true) {
                this->setFirstFrame(false);
                this->m_remoteCEndpoint = multicastEndpoint;
            }

            //parse data
            parseData(pBuffer, this->m_vctMarkData, this->m_vctRigidbodyData);

            //clear buffer
            EmptyBuffer(pBuffer, MAX_FRAME_SIZE);
        }while (this->m_TransmissionSocket.available() > 0);

        ReleaseBuffer(pBuffer);
    }
    //parse marker and rigibody data
    void MotionCaptureFZMotion::parseData(const byte* const pData, vector<LMarker>& allMarkers, vector<LRigidBody>& allRigidBodys) {
        //parse received data
        byte* ptr = const_cast<byte*>(pData);

        Message eMessage;
        CopyBuffer((byte*)&eMessage, ptr, sizeof(Message));

        if (eMessage != Message::MotionCaptureData)
            return;

        ptr += sizeof(Message);
        uint16 uDataBytes;
        CopyBuffer((byte*)&uDataBytes, ptr, sizeof(uint16));
        ptr += sizeof(uint16);
        CopyBuffer((byte*)&this->m_uFrameNumber, ptr, sizeof(uint32));
        ptr += sizeof(int32);
        uint32 uMarkerSets;
        CopyBuffer((byte*)&uMarkerSets, ptr, sizeof(uint32));
        ptr += sizeof(uint32);
        int32 iMarkerNumber;
        CopyBuffer((byte*)&iMarkerNumber, ptr, sizeof(int32));
        ptr += sizeof(int32);

        allMarkers.clear();
        if (iMarkerNumber > 0) {
            uint32 uCopySize = sizeof(LMarker) * iMarkerNumber;
            allMarkers.resize(iMarkerNumber);
            CopyBuffer((byte*)allMarkers.data(), ptr, uCopySize);
            ptr += uCopySize;
        }

        allRigidBodys.clear();
        uint32 uRigidSets;
        CopyBuffer((byte*)&uRigidSets, ptr, sizeof(uint32));
        ptr += 4;
        if (uRigidSets > 0) {
            uint32 uCopySize = sizeof(LRigidBody) * uRigidSets;
            allRigidBodys.resize(uRigidSets);
            CopyBuffer((byte*)allRigidBodys.data(), ptr, uCopySize);
            ptr += uCopySize;
        }
    }
    void MotionCaptureFZMotion::waitForNextFrame() {
        s_mutex.lock();

        if (this->isConnected() == true) {
            receiveFrameData();
        }
        s_mutex.unlock();
    }
    const std::map<std::string, RigidBody>& MotionCaptureFZMotion::rigidBodies() const {
        s_mutex.lock();
        if (this->m_uPreviousFrame == this->m_uFrameNumber) {
            s_mutex.unlock();
            return rigidBodies_;
        }

        rigidBodies_.clear();
       
        for (auto& lrb : this->m_vctRigidbodyData) {
            auto& tag = m_mapRigidbodyTagList[lrb.ID];
            Eigen::Vector3f position(
                lrb.sPosition.x + tag.sCenteroidTransform.x,
                lrb.sPosition.y + tag.sCenteroidTransform.y,
                lrb.sPosition.z + tag.sCenteroidTransform.z
            );
            Eigen::Quaternionf rotation(Eigen::Quaternionf(
                lrb.sOrientation.qw,
                lrb.sOrientation.qx,
                lrb.sOrientation.qy,
                lrb.sOrientation.qz
            ));
            RigidBody rigidbody(tag.szName, position, rotation);
            rigidBodies_.emplace(tag.szName, rigidbody);
        }

        this->m_uPreviousFrame = this->m_uFrameNumber;
        s_mutex.unlock();
        return rigidBodies_;
    }
    const PointCloud& MotionCaptureFZMotion::pointCloud() const {
        s_mutex.lock();
        if (this->m_uPreviousFrame == this->m_uFrameNumber) {
            s_mutex.unlock();
            return pointcloud_;
        }

        if (pointcloud_.size() != this->m_vctMarkData.size()) {
            pointcloud_.resize(this->m_vctMarkData.size(), Eigen::NoChange);
        }
        
        for (uint32 row = 0; row < this->m_vctMarkData.size(); row++) {
            auto& marker = this->m_vctMarkData[row];
            pointcloud_.row(row) << marker.sPosition.x, marker.sPosition.y, marker.sPosition.z;
        }
        s_mutex.unlock();
        return pointcloud_;
    }
}
