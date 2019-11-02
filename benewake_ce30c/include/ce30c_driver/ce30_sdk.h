#include "undistort.h"
extern "C"
{
    namespace benewake
    {
        double gCameraMatrix[9], gDistCoeffs[4];
        unsigned short gRawDistMatrix[HEIGHT * WIDTH] = {0}, gRawAmpMatrix[HEIGHT * WIDTH] = {0};

        // @brief Establish Ethernet connection and initiallize LiDAR.
        // @param _sd Socket handle.
        // @param _ip IP address of LiDAR.
        bool initDevice(int &_sd, char *_ip);

        // @brief Start several times measurement
        // @param _sd Socket handle.
        // @param _times The times of measurement. When set to 0, the measurement will be continuous
        // untill the LiDAR receive stop measurement command. Otherwise, the measurement will
        // automatically stop when the times of measurement is achieved.
        bool startMeasurement(int _sd, int _times);

        // @brief Get distance and signal strength of LiDAR's FoV.
        // @param _sd Socket handle.
        // @param _dist Distance data buffer, the size should be 24[height] * 660[width].
        // @param _amp Distance data buffer, the size should be 24[height] * 660[width].
        // The data sequence is from field of view's left to right and then from top to bottom.
        bool getDistanceData(int _sd, unsigned short *_dist, unsigned short *_amp);

        // @brief Stop measurement.
        // @param _sd Socket handle.
        bool stopMeasurement(int _sd);

        // @brief Stop TCP communication.
        // @param _sd Socket handle.
        bool closeDevice(int &_sd);

        // @brief Transform distance data into point cloud .
        // @param _depth Distance data buffer.
        // @param _coordX x-axis coordinates of point cloud, the direction is from left to right.
        // @param _coordY y-axis coordinates of point cloud, the direction is from bottom to top.
        // @param _coordZ z-axis coordinates of point cloud, the direction is from near to far.
        // All params should have the same size of 24[height] * 660[width], and the same sequence
        // number params construct a space point (_coordX[n], _coordY[n], _coordZ[n]).
        // The data sequence is from field of view's left to right and then from top to bottom.
        bool getPointCloud(unsigned short *_dist, float *_coordX, float *_coordY, float *_coordZ);

        // @brief Change LiDAR's IP Address.
        // @param _sd Socket handle.
        // @param _newIP New IP address.
        bool changeIPAddress(int _sd, char *_newIP);
    }
}
