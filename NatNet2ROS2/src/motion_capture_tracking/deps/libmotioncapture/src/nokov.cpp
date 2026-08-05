#include "libmotioncapture/nokov.h"

#include <string>
#include <thread>
#include <mutex>   
#include <unordered_map>
#include <Eigen/Geometry> 
//#include <ros/ros.h>
#include "NokovSDKCAPI.h"
// NOKOV
#include "NokovSDKClient.h"

namespace libmotioncapture {

    typedef struct
    {
        int ID;                                 // RigidBody identifier
        float x, y, z;                          // Position
        float qx, qy, qz, qw;                   // Orientation
    } sBodyData;

    typedef struct
    {
        int iFrame;                                     // host defined frame number
        int nOtherMarkers;                              // # of undefined markers
        MarkerData OtherMarkers[MAX_MARKERS];           // undefined marker data
        int nRigidBodies;                               // # of rigid bodies
        sBodyData RigidBodies[MAX_RIGIDBODIES];    // Rigid body data
        float fLatency;                                 // host defined time delta between capture and send
        unsigned int Timecode;                          // SMPTE timecode (if available)
        unsigned int TimecodeSubframe;                  // timecode sub-frame data
        long long iTimeStamp;                           // FrameGroup timestamp
        short params;                                   // host defined parameters

    } sFrameOfObjData;

    // Global Var
    sFrameOfObjData frameObjData = {0};
    std::mutex mtx;

    const sFrameOfObjData& GetCurrentFrame()
    {
        static sFrameOfObjData frame = {0};
        {
            std::lock_guard<std::mutex> lck (mtx);
            frame = frameObjData;
        }
        return frame;
    }

    void DataHandler(sFrameOfMocapData* pFrameOfData, void* pUserData)
    {
        if (nullptr == pFrameOfData)
            return;

        // Store the frame
        std::lock_guard<std::mutex> lck (mtx);

        int nmaker = (pFrameOfData->nOtherMarkers < MAX_MARKERS)?pFrameOfData->nOtherMarkers:MAX_MARKERS;

        frameObjData.iFrame = pFrameOfData->iFrame;
        frameObjData.nOtherMarkers = nmaker;
        frameObjData.nRigidBodies = pFrameOfData->nRigidBodies;
        frameObjData.fLatency = pFrameOfData->fLatency;
        frameObjData.Timecode = pFrameOfData->Timecode;
        frameObjData.TimecodeSubframe = pFrameOfData->TimecodeSubframe;
        frameObjData.iTimeStamp = pFrameOfData->iTimeStamp;
        frameObjData.params = pFrameOfData->params;
        
        for(int i = 0; i< nmaker; ++i)
        {
            frameObjData.OtherMarkers[i][0] = pFrameOfData->OtherMarkers[i][0] * 0.001;
            frameObjData.OtherMarkers[i][1] = pFrameOfData->OtherMarkers[i][1] * 0.001;
            frameObjData.OtherMarkers[i][2] = pFrameOfData->OtherMarkers[i][2] * 0.001;          
        }

        for(int i = 0; i< pFrameOfData->nRigidBodies; ++i)
        {
            frameObjData.RigidBodies[i].ID =  pFrameOfData->RigidBodies[i].ID;
            frameObjData.RigidBodies[i].x =  pFrameOfData->RigidBodies[i].x * 0.001;
            frameObjData.RigidBodies[i].y =  pFrameOfData->RigidBodies[i].y * 0.001;
            frameObjData.RigidBodies[i].z =  pFrameOfData->RigidBodies[i].z * 0.001;
            frameObjData.RigidBodies[i].qx =  pFrameOfData->RigidBodies[i].qx;
            frameObjData.RigidBodies[i].qy =  pFrameOfData->RigidBodies[i].qy;
            frameObjData.RigidBodies[i].qz =  pFrameOfData->RigidBodies[i].qz;
            frameObjData.RigidBodies[i].qw =  pFrameOfData->RigidBodies[i].qw;
        }
    }

    class MotionCaptureNokovImpl
    {
    public:
        std::string version = "0.0.0.0";
        int updateFrequency = 100;
        bool enableFixedUpdate = false;
        sDataDescriptions* pBodyDefs = nullptr;
        NokovSDKClient* pClient = nullptr;
        int lastFrame = 0;
        std::unordered_map<std::string, size_t> bodyMap;

        size_t GetBodyIdByName(const std::string& name) const {
            if (bodyMap.find(name) != bodyMap.end())
            {
                return bodyMap.at(name);
            }

            return -1;
        }

        const std::string GetBodyNameById(size_t id) const {
            for (auto pair : bodyMap)
            {
                if (pair.second == id)
                {
                    return pair.first;
                }
            }

            return std::string();
        }

        ~MotionCaptureNokovImpl()
        {
            if (nullptr != pClient)
            {
                pClient->Uninitialize();
                delete pClient;
                pClient = nullptr;
            }

            if (nullptr != pBodyDefs)
            {
                XingYing_FreeDescriptions(pBodyDefs);
                pBodyDefs= nullptr;
            }
        }
    };

    MotionCaptureNokov::MotionCaptureNokov(
        const std::string& hostname,
        bool enableFrequency, 
        int updateFrequency)
    {
        pImpl = new MotionCaptureNokovImpl;

        NokovSDKClient* theClient = new NokovSDKClient();
        unsigned char version[4] = {0};
        theClient->NokovSDKVersion(version);
        {
            std::stringstream sstr;
            sstr << (int)version[0] << "." << (int)version[1] << "." << (int)version[2] << "." << (int)version[3];
            pImpl->version = sstr.str();
        }

        theClient->SetDataCallback(DataHandler);

        // Check the ret value
        int retValue = theClient->Initialize((char*)hostname.c_str());
        if (ErrorCode_OK != retValue)
        {
            std::stringstream sstr;
            sstr << "Error connecting XINGYING on address: " << hostname << " Code:" << retValue;
            throw std::runtime_error(sstr.str());
        }

        if (ErrorCode_OK != theClient->GetDataDescriptions(&pImpl->pBodyDefs) || nullptr == pImpl->pBodyDefs)
        {
			std::stringstream sstr;
			sstr << "Error Request BodyDefs on address: " << hostname << " Code:" << retValue;
			throw std::runtime_error(sstr.str());
        }
        
        for (int iDataDef = 0; iDataDef < pImpl->pBodyDefs->nDataDescriptions; ++iDataDef)
        {
            if (pImpl->pBodyDefs->arrDataDescriptions[iDataDef].type == Descriptor_RigidBody)
            {
                auto bodeDef = pImpl->pBodyDefs->arrDataDescriptions[iDataDef].Data.RigidBodyDescription;
                pImpl->bodyMap[bodeDef->szName] = bodeDef->ID;
            }
        }

        pImpl->pClient = theClient;
        pImpl->enableFixedUpdate = enableFrequency;
        pImpl->updateFrequency = updateFrequency;
    }

    MotionCaptureNokov::~MotionCaptureNokov()
    {
        if (pImpl)
        {
            delete pImpl;
        }
    }

    const std::string& MotionCaptureNokov::version() const
    {
        return pImpl->version;
    }

    void MotionCaptureNokov::waitForNextFrame()
    {
        static auto lastTime = std::chrono::high_resolution_clock::now();
        auto now = std::chrono::high_resolution_clock::now();

        if (pImpl->enableFixedUpdate)
        {
            auto elapsed = now - lastTime;
            auto desiredPeriod = std::chrono::milliseconds(1000 / pImpl->updateFrequency);
            //std::cout <<"elapsed: " << std::chrono::duration<double>(elapsed).count() << "\tdesired:" << std::chrono::duration<double>(desiredPeriod).count() << std::endl;
            if (elapsed < desiredPeriod) {
                //std::cout << "Sleep Done" << std::endl;
                std::this_thread::sleep_for(desiredPeriod - elapsed);
            }
        }

        int frameNo;
        do
        {
            frameNo = GetCurrentFrame().iFrame;
        }
        while (frameNo == pImpl->lastFrame);
        pImpl->lastFrame = frameNo;  
        lastTime = std::chrono::high_resolution_clock::now();
    }

	bool MotionCaptureNokov::supportsPointCloud() const
	{
		return true;
	}

	const std::map<std::string, libmotioncapture::RigidBody>& MotionCaptureNokov::rigidBodies() const
	{
        rigidBodies_.clear();

		auto frameData = GetCurrentFrame();
		for (int iBody = 0; iBody < frameData.nRigidBodies; ++iBody) {

			const auto& rb = frameData.RigidBodies[iBody];
			Eigen::Vector3f position(
				rb.x,
				rb.y,
				rb.z);

			// Convention
			Eigen::Quaternionf rotation(rb.qw, rb.qx, rb.qy, rb.qz);
            auto bodyName = pImpl->GetBodyNameById(rb.ID);

            rigidBodies_.emplace(bodyName, RigidBody(bodyName,position, rotation));
		}

        return rigidBodies_;
	}


	libmotioncapture::RigidBody MotionCaptureNokov::rigidBodyByName(const std::string& name) const
	{
        size_t bodyId = pImpl->GetBodyIdByName(name);
        if (bodyId < 0)
        {
            throw std::runtime_error("Unknown rigid body!");
        }

		auto frameData = GetCurrentFrame();
		for (size_t iBody = 0; iBody < frameData.nRigidBodies; ++iBody) {

			const auto& rb = frameData.RigidBodies[iBody];

			if (bodyId == rb.ID)
			{
				Eigen::Vector3f position(
					rb.x,
					rb.y,
					rb.z);

		        Eigen::Quaternionf rotation(rb.qw, rb.qx, rb.qy, rb.qz);

                return RigidBody(pImpl->GetBodyNameById(bodyId), position, rotation);
			}
		}

        throw std::runtime_error("Unknown rigid body!");
	}

	const libmotioncapture::PointCloud& MotionCaptureNokov::pointCloud() const
	{
		auto frameData = GetCurrentFrame();
        size_t count = frameData.nOtherMarkers;
		pointcloud_.resize(count, Eigen::NoChange);
		for (size_t iMarkerIdx = 0; iMarkerIdx < count; ++iMarkerIdx) {

			pointcloud_.row(iMarkerIdx) << frameData.OtherMarkers[iMarkerIdx][0],
                frameData.OtherMarkers[iMarkerIdx][1],
                frameData.OtherMarkers[iMarkerIdx][2];
		}
		return pointcloud_;
	}


	bool MotionCaptureNokov::supportsRigidBodyTracking() const
	{
        return true;
	}

}
