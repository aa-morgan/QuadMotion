/*
 *  Class - QuadMotionApp
 *  Author - Alex Morgan
 *      Starting point for the Cinder Application
 */

// Include the necessary cinder libraries
#include "Resources.h"
#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/TextureFont.h"
#include "cinder/ip/Threshold.h"
#include "cinder/ip/EdgeDetect.h"
#include "GUI.h"
#include "TrackerListener.h"
#include "TrackerFinder.h"
#include "Quadcopter.h"
#include <unistd.h>

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace std::chrono;
using namespace Leap;

/*
 *  Declaration of Tracker App
 */
class QuadMotionApp : public AppBasic {
public:
	void setup();
    void update();
	void draw();
    void drawTrack3();
    void drawTrack4();
    void draw3D();
    void drawGeneral();
    void drawSeperator();
    void prepareSettings( Settings *settings );
    
private:
    // Class to manage GUI aspects of the App
    GUI mGUI;
    // Class manage everything else to do with:
    //  - Communication with the LeapMotion controller
    //  - Tracking the Quadcopter
    //  - Calculation of the PID controls
    //  - Communication with the Arduino
    Quadcopter mQuadcopter;
    
    // Define Fonts of the App
    Font mFont;
    gl::TextureFontRef mTextureFont;
};

/*
 *  Prepare Settings - Called Once.
 */
void QuadMotionApp::prepareSettings( Settings *settings ){
    settings->setWindowSize( 640+(2*240), (2*240)+200 );
    //settings->setFrameRate( 30.0 );
    settings->setTitle( "QuadMotion - Diagnosis Program" );
}

/*
 *  Setup - Called Once
 */
void QuadMotionApp::setup()
{
    // Initialise the GUI
    mGUI = GUI();
    
    // Initialise the Quadcopter
    mQuadcopter = Quadcopter();
    mQuadcopter.setup();
    
    // Initialise the Font
    mFont = Font( "AmericanTypewriter", 22 );
    mTextureFont = gl::TextureFont::create( mFont );
    gl::enableAlphaBlending( false );
    
    //console() << std::setprecision(4) << std::fixed;
    console() << "Setup Complete!" << endl;
}

/*
 *  Update - Called on every Frame - Called First
 */
void QuadMotionApp::update()
{
    // Update the Quadcopter
    mQuadcopter.update();
    
    // Update the GUI Data
    mGUI.updateData( mQuadcopter, mTextureFont );
}

/*
 *  Draw - Called on every Frame - Called Second
 */
void QuadMotionApp::draw()
{
    // Update the GUI - Mode 0 (Default)
    mGUI.draw( 0 );
}

// This line tells Cinder to actually create the application
CINDER_APP_BASIC( QuadMotionApp, RendererGl )