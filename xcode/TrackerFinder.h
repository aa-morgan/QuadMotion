#ifndef __TrackDots__TrackerFinder__
#define __TrackDots__TrackerFinder__

#include "Resources.h"
#include "CinderOpenCV.h"
#include "cinder/ImageIo.h"
#include "cinder/app/AppBasic.h"

using namespace Leap;
using namespace std;
using namespace ci;
using namespace ci::app;

/*
 *
 */
class TrackerFinder {
public:
    TrackerFinder();
    TrackerFinder(int);
    void setTrackNum( int );
    
    vector< vector< vector<float> > > findDots( vector<Image>, vector<Surface> );
    
    void processOpenCV( Surface, int );
    void setProcessedSurface( Surface, int );
    Surface getProcessedSurface( int );
    vector< Surface > getProcessedSurfaces();
    
    bool lastFrameSuccess();
    
    vector< vector<float> > getOrigins();
    vector<float> getDirections();
    
private:
    
    vector< cv::KeyPoint > findSimpleBlobs( Surface, int );
    vector< vector<float> > findHoughCircles( Surface, int );
    void pairDots( vector<Image>& );
    void find3DPositions( vector<Image>& );
    
    void find2DOrigins( vector<Image>& );
    float findAngle( vector<float> pOrig, vector<float> pPoint );
    void postProcessing2D( vector<Image>& );
    void postProcessing3D( vector<Image>& );
    float myMod( float, int );
    
    int mNumTrack;
    bool mLastFrameSuccess;
    int mPredictIndex;
    
    vector< vector< vector<float> > > mDotsTrio; // Positions of all dots in both 2D Cameras and in 3D
    vector< vector<float> > mOrigins; // Origins in both 2D Cameras and in 3D
    vector<float> mDirections; // Directions in both 2D Cameras and in 3D
    
    vector< Surface > currentProcessedSurface = { Surface(), Surface() };
    bool processedSurfaceSet[2] = {false};

};

#endif /* defined(__TrackDots__TrackerFinder__) */
