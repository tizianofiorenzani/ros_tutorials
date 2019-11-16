#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string>
#include <cstring>
#include <unistd.h>
#include <math.h>
//#define DEBUG
#ifdef DEBUG
#include <fstream>
#endif // DEBUG

#define WIDTH       320
#define HEIGHT      24
#define TOTALDATA   2 * WIDTH * HEIGHT
//#define DEBUG
//#define PRINTDATA

// define command buff
const char kStart[51]       = "getDistanceAndAmplitudeSorted                     "; // start command
const char kStop[51]        = "join                                              "; // stop command
const char kDisconnect[51]  = "disconnect                                        "; // disconnect command
const char kGreyImage[51]   = "enableFeatures 131072                             "; // enable grey image
const char kSetIntegTime[51]= "setIntegrationTime3D 1600                         "; // integration time 1600
const char kSetROI[51]      = "roi 0 0 3                                         ";


// Ethernet connect
int TCP_connect(int &_sd, char *_ip){
 	int port = 50660;

 	// CE_TCP
	_sd = socket(AF_INET, SOCK_STREAM, 0);
	if (_sd < 0){
		// printf("Socket error!\n");
		return -1;
	}
	struct sockaddr_in ser_addr = {0};
	ser_addr.sin_family = AF_INET;
	ser_addr.sin_port = htons(port);
	const char* c_addr = _ip;
	ser_addr.sin_addr.s_addr = inet_addr(c_addr);

  // start connetion
	int ret = 0, total = 0;
	unsigned long ul = 1;
	ret = ioctl(_sd, FIONBIO, (unsigned long*)&ul);
	if(ret == -1){
        close(_sd);
        return 0;
	}
	// printf("Device connecting ...\n");
	ret = connect(_sd, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
	if(ret && errno != EINPROGRESS){
        return 0;
	}
	if(ret == 0){
		// printf("Connected!\n");
	}
	else{
		struct timeval tv;
		fd_set w;
		FD_ZERO(&w);
		FD_SET(_sd, &w);
		tv.tv_sec = 7;
		tv.tv_usec = 0;
		int retval = select(_sd+1, 0, &w, 0, &tv);
		if(retval == -1){
		    // select error
            return -1;
		}
		else if(retval == 0){
		    // connect timeout
            return -2;
		}
		else{
            int er;
            socklen_t len = sizeof(er);
            if(getsockopt(_sd, SOL_SOCKET, SO_ERROR, (char*)&er, &len) < 0){
                // getsocketopt error
                return -3;
            }
            if(er != 0){
                // connnect error
                return -4;
            }
		}
	}
	ul = 0;
	ret = ioctl(_sd, FIONBIO, (unsigned long*)&ul);
	if(ret == -1){
        // ioctl block error
        close(_sd);
        return -5;
	}
	struct timeval timeout = {3, 0};
	setsockopt(_sd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
	setsockopt(_sd, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));

	return 1;
}

// receive data
int recv_data(int _sd, unsigned char *_buff, int _length){
	int total = 0, ret = 0;
	while(total != _length){
		ret = recv(_sd, _buff + total, (_length - total), 0);
		if(ret < 0){
			// printf("Data receiving failed!\n");
			return -1;
		}
#ifdef PRINTDATA
		else{
			// printf("Receiving bytes: %d\n", ret);
		}
#endif // PRINTDATA
		total += ret;
	}

#ifdef PRINTDATA
	printf("Data: %02x, %02x, %02x, %02x, ...\n", _buff[0], _buff[1], _buff[2], _buff[3]);
#endif // PRINTDATA
	return total;
}

// send command
int send_command(int _sd, const char *_command, int _length){
    int total = 0, ret = 0;
    do
    {
        ret = send(_sd, _command + total, (_length - total), 0);
        if(ret < 0)
            return false;
        total += ret;
    }while(total != _length);
	return total;
}

// translate char* to int
int char_to_int(char *_c)
{
    int value = 0;
    int len = strlen(_c);
    if(len < 0)
        return 0xffffffff;

    for(int i = len - 1; i >= 0; i --)
    {
        if(i == 0 && _c[0] == '-')
        {
            value *= -1;
        }
        else if(_c[i] >= '0' && _c[i] <= '9')
        {
            value += (int)(_c[i] - '0') * pow(10.0, (len - 1 - i));
        }
        else
        {
            return 0xffffffff;
        }
    }

    return value;
}

// clear socket buffer
void clearSocketBuffer(int _fd)
{
    struct timeval tmOut;
    tmOut.tv_sec = 0;
    tmOut.tv_usec = 200000;
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(_fd, &fds);
    int nRet;
    char tmp[2];
    memset(tmp, 0, sizeof(tmp));
    while(1)
    {
        nRet = select(FD_SETSIZE, &fds, NULL, NULL, &tmOut);
        if(nRet == 0)
            break;
        recv(_fd, tmp, 1, 0);
    }
}

