#ifndef __TrackDotsCV__PIDcontroller__
#define __TrackDotsCV__PIDcontroller__

#include "Resources.h"
#include <ctime>

using namespace Leap;
using namespace std;
using namespace std::chrono;
using namespace ci;
using namespace ci::app;

/*
 *
 */
class PIDcontroller {
public:
    PIDcontroller();
    
    void update( vector<float> );
    vector<float> caclulateControlSignal( bool );
    
    vector<float> getReferencePosition();
    void updateReferencePosition( vector<float> );
    
    vector< vector<float> > getControlSignalComponents();
    
private:
    
    void updateTime();
    
    vector<float> getCurrentPosition();
    void updateCurrentPosition( vector<float> );
    
    vector<float> getErrorInPosition();
    void updateErrorInPosition();
    
    vector<float> getRunningErrorInPosition();
    void updateRunningErrorInPosition();
    
    vector<float> getVelocity();
    void updateVelocity( vector<float> );
    
    vector< vector<float> > getPID();
    void updatePID( vector< vector<float> > );
    
    vector<float> sanatise( vector<float> );
    
    vector< vector<float> > mPID;
    float mPreviousTime;
    float mDeltaTime;
    float mThresholdTimeout;
    high_resolution_clock::time_point mPrev;
    high_resolution_clock::time_point mNow;
    
    vector<float> mReferencePosition;
    vector<float> mCurrentPosition;
    vector<float> mErrorInPosition;
    vector<float> mRunningErrorInPosition;
    vector<float> mVelocity;
    
    vector< vector<float> > mRunningErrorInPositionWindow;
    int mRunningErrorInPositionWindowSize;
    int mRunningErrorInPositionWindowIndex;
    
    vector< vector<float> > mControlSignalComponents;
    
    bool mFirstFrame;
    
    // X, Y, Z : Left-Right, Up-Down, Forward-Back
    vector< vector<float> > mLimits;
    
};

#endif /* defined(__TrackDotsCV__PIDcontroller__) */
