#ifndef __TrackDotsCV__Quadcopter__
#define __TrackDotsCV__Quadcopter__

#include "Resources.h"
#include "cinder/gl/Texture.h"
#include "CinderOpenCV.h"
#include "TrackerFinder.h"
#include "TrackerListener.h"
#include "PIDcontroller.h"
#include "Arduino.h"

using namespace Leap;
using namespace std;
using namespace ci;
using namespace ci::app;

/*
 *
 */
class Quadcopter {
public:
    Quadcopter();
    
    void setup();
    void update();
    vector< vector< vector<float> > > get2DPositions();
    vector< vector<float> > get2DCenters();
    vector< vector<float> > get3DPositions();
    vector<float> get3DCenter();
    bool getFrameSuccess();
    
    vector< gl::Texture > getCameraImages();

    vector<float> getControlSignal( bool );
    void sendControlSignal( bool );
    
    bool isLeapMotionConnected();
    bool isArduinoConnected();
    PIDcontroller& getPIDcontroller();
    TrackerFinder& getTrackerFinder();
    
private:
    
    TrackerFinder mTrackerFinder;
    TrackerListener mTrackerListener;
    Controller mTrackerController;
    PIDcontroller mPIDcontroller;
    Arduino mArduino;
    
    bool mLeapMotionConnected = false;
    bool mArduinoConnected = false;
    
    vector< vector< vector<float> > > mDotsFound;
    
    int mNumTrack = NUM_TRACK;
    
};

#endif /* defined(__TrackDotsCV__Quadcopter__) */
