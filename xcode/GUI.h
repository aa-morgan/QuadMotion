#ifndef __TrackDotsCV__GUI__
#define __TrackDotsCV__GUI__

#include "Resources.h"
#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/TextureFont.h"
#include "Quadcopter.h"
#include <unistd.h>

using namespace std;
using namespace std::chrono;
using namespace ci;
using namespace ci::app;

/*
 *
 */
class GUI {
public:
    GUI();
    void updateData( Quadcopter&, gl::TextureFontRef& );
    void draw( int );
    void drawCameras( int, int );
    void draw3D( int, int, int );
    void drawGeneral( int, int, int, int );
    void drawArrow( vector<float>, float, float, int, int, bool );
    void drawCross( vector<float>, int, int );
    
private:
    Quadcopter mQuadcopter;
    vector< vector< vector<float> > > m2DPositions;
    vector< vector<float> > m2DCenters;
    vector< vector<float> > m3DPositions;
    vector<float> m3DCenter;
    vector< gl::Texture > mProcessedImageTex;
    gl::TextureFontRef mTextureFont;
    
    int mFPS;
    high_resolution_clock::time_point tPrev;
    high_resolution_clock::time_point tNow;
    
    float findAngle( vector<float> pOrig, vector<float> pPoint );
    
};


#endif /* defined(__TrackDotsCV__GUI__) */
