#ifndef __TrackDotsCV__Arduino__
#define __TrackDotsCV__Arduino__

#include "Resources.h"
#include "cinder/Serial.h"
#include "cinder/Text.h"

using namespace std;
using namespace ci;
using namespace ci::app;

#define CONNECT_PACKET 251
#define CONTROL_PACKET 252
#define SETTING_PACKET 253

/*
 *
 */
class Arduino {
public:
    Arduino();
    
    void setup();
    void send( vector<float> );
    
private:
    
    void sendRaw( vector<int> );
    
	Serial mSerial;
    int mBaudRate = 115200;
    
    bool mSetupSucess = false;
    
    vector<int> mNeutral = { 0, 127, 127, 127 };
    int mMid = 127;
    int mRange = 50; // From Mid

};

#endif /* defined(__TrackDotsCV__Arduino__) */
