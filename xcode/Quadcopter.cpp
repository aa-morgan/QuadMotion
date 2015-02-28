/*
 *  Class - Quadcopter
 *  Author - Alex Morgan
 */

#include "Quadcopter.h"

/*
 *  Blank Constructor
 */
Quadcopter::Quadcopter() {
}

/*
 *  Setup -
 */
void Quadcopter::setup() {
    
    // Start LeapMotion Listener and Controller
    mTrackerListener = TrackerListener();
    mTrackerController = Controller();
    
    // Allow Camera Images to be read
    mTrackerController.setPolicyFlags(Leap::Controller::POLICY_IMAGES);
    
    // Have the sample listener receive events from the controller
    mTrackerController.addListener(mTrackerListener);
    mLeapMotionConnected = true;
    
    // Initialise the Tracker Finder and set the Number of Dots to track, 4
    mTrackerFinder = TrackerFinder( mNumTrack );
    
    // Initialise the PID Controller
    mPIDcontroller = PIDcontroller();
    
    // Set Reference Position
    //    Up-Down, Rotation, Forward-Back, Left Right
    mPIDcontroller.updateReferencePosition( vector<float> { 600.0, 180.0, 0.0, 0.0 } );
    
    // Init & Setup Arduino for physical control of Quadcopter
    if ( USE_ARDUINO ) {
        mArduino = Arduino();
        mArduino.setup();
        mArduinoConnected = true;
    }
    
    
}

/*
 *  Update - 
 */
void Quadcopter::update() {
    
    // Find Dots in Camera Images, and update mDotsFound. If didn't find expected Dots, returns empty.
    mDotsFound = mTrackerFinder.findDots( mTrackerListener.getImages(), mTrackerListener.getSurfaces() );
    
    if ( mTrackerFinder.lastFrameSuccess() ) {
        // Throttle     (Up-Down)                   -  Y
        // Rudder       (Clockwise-AntiClockwise)   -  Dir
        // Elevator     (Forward-Back)              -  Z
        // Aileron      (Left-Right)                -  X
        mPIDcontroller.update( vector<float> { get3DCenter()[1], get3DCenter()[3], get3DCenter()[2], get3DCenter()[0] } );
        
        // If using an Arduino, send PID control signals, else just print them
        if (  USE_ARDUINO ) sendControlSignal( true );
        if ( !USE_ARDUINO ) getControlSignal( true );
    }
    
}

/*
 *  Get 2D Positions -
 */
vector< vector< vector<float> > > Quadcopter::get2DPositions() {
    
    // Store 2D positions
    vector< vector< vector<float> > > t2DPositions =
        vector< vector< vector<float> > >( mDotsFound.size() , vector< vector<float> >( 2, vector<float>(2) ) );
    
    // Extract just 2D positons from mDotsFound
    for ( int tDot = 0; tDot < mDotsFound.size(); tDot++ ) {
        for ( int tCam = 0; tCam < 2; tCam++ ) {
            t2DPositions[ tDot ][ tCam ][ 0 ] = mDotsFound[ tDot ][ tCam ][ 0 ];
            t2DPositions[ tDot ][ tCam ][ 1 ] = mDotsFound[ tDot ][ tCam ][ 1 ];
        }
    }
    
    // Return 2D positions
    return t2DPositions;
}

/*
 *  Get 2D Centers -
 */
vector< vector<float> > Quadcopter::get2DCenters() {
    
    vector< vector<float> > tCenters ( 3, vector<float>(2, 0.0) );
    vector< vector<float> > tOrigins ( 3, vector<float>(3, 0.0) );
    vector<float> tDirections (3, 0.0);
    tOrigins = mTrackerFinder.getOrigins();
    tDirections = mTrackerFinder.getDirections();

    tCenters[0] = tOrigins[0];
    tCenters[1] = tOrigins[1];
    tCenters[2] = { tDirections[0], tDirections[1] };
    
    return tCenters;
}

/*
 *  Get 3D Positions -
 */
vector< vector<float> > Quadcopter::get3DPositions() {
    
    // Store 3D positions
    vector< vector<float> > t3DPositions = vector< vector<float> >( mDotsFound.size(), vector<float>(3) );
    
    // Camera Number for 3D positions
    int tCam = 2;
    
    // Extract just 3D positons from mDotsFound
    for ( int tDot = 0; tDot < mDotsFound.size(); tDot++ ) {

        t3DPositions[ tDot ][ 0 ] = mDotsFound[ tDot ][ tCam ][ 0 ];
        t3DPositions[ tDot ][ 1 ] = mDotsFound[ tDot ][ tCam ][ 1 ];
        t3DPositions[ tDot ][ 2 ] = mDotsFound[ tDot ][ tCam ][ 2 ];
    }
    
    // Return 3D positions
    return t3DPositions;
}

/*
 *  Get 3D Center -
 */
vector<float> Quadcopter::get3DCenter() {
    
    vector<float> tCenter ( 4, 0.0 );
    vector< vector<float> > tOrigins ( 3, vector<float>(3, 0.0) );
    vector<float> tDirections (3, 0.0);
    tOrigins = mTrackerFinder.getOrigins();
    tDirections = mTrackerFinder.getDirections();
    
    tCenter[0] = tOrigins[2][0];
    tCenter[1] = tOrigins[2][1];
    tCenter[2] = tOrigins[2][2];
    tCenter[3] = tDirections[2];
    
    return tCenter;
}


/*
 *  Get Camera Images -
 */
vector< gl::Texture > Quadcopter::getCameraImages() {
    
    vector< Surface > tSurfaces = mTrackerFinder.getProcessedSurfaces();
    
    return vector< gl::Texture > { gl::Texture( tSurfaces[0] ), gl::Texture( tSurfaces[1] ) };
}

/*
 *  Get Control Signal -
 *    QuadCopter Frame of Reference:
 *      Throttle - Fly Up & Down
 *      Rudder   - Rotate Clockwise & Anti-Clockwise
 *      Elevator - Pitch Forward & Backwards
 *      Aileron  - Roll Left & Right
 */
vector<float> Quadcopter::getControlSignal( bool pPrint ) {
    
    // Calculate Control Signal from PID Controller and send it to Arduino
    vector<float> tControlSig = mPIDcontroller.caclulateControlSignal( pPrint );
    
    // Temp variables
    vector<float> tTemp ( 2, 0.0 );
    
    // Convert from Leap Motion Frame of Reference to QuadCopter Frame of Reference
    float tAngle = get3DCenter()[3] * ( PI / 180.0 );
    tTemp[0] = ( tControlSig[2] * -cos( tAngle ) ) + ( tControlSig[3] * -sin( tAngle ) );
    tTemp[1] = ( tControlSig[3] * -cos( tAngle ) ) + ( tControlSig[2] * -sin( tAngle ) );
    tControlSig[2] = tTemp[0];
    tControlSig[3] = tTemp[1];
    
    return tControlSig;
}

/*
 *  Send Control Signal -
 *    QuadCopter Frame of Reference:
 *      Throttle - Fly Up & Down
 *      Rudder   - Rotate Clockwise & Anti-Clockwise
 *      Elevator - Pitch Forward & Backwards
 *      Aileron  - Roll Left & Right
 */
void Quadcopter::sendControlSignal( bool pPrint ) {
    
    // Calculate Control Signal from PID Controller and send it to Arduino
    vector<float> tControlSig = mPIDcontroller.caclulateControlSignal( pPrint );
    
    // Temp variables
    vector<float> tTemp ( 2, 0.0 );
    
    // Convert from Leap Motion Frame of Reference to QuadCopter Frame of Reference
    float tAngle = get3DCenter()[3] * ( PI / 180.0 );
    tTemp[0] = ( tControlSig[2] * -cos( tAngle ) ) + ( tControlSig[3] * -sin( tAngle ) );
    tTemp[1] = ( tControlSig[3] * -cos( tAngle ) ) + ( tControlSig[2] * -sin( tAngle ) );
    tControlSig[2] = tTemp[0];
    tControlSig[3] = tTemp[1];
    
    // Fix Throttle
    //tControlSig[0] = 0.2;
    
    // Send Control Signal to Arduino
    mArduino.send( tControlSig );
    
}

/*
 *
 */
bool Quadcopter::getFrameSuccess() {
    return mTrackerFinder.lastFrameSuccess();
}

/*
 *
 */
bool Quadcopter::isLeapMotionConnected() {
    return mLeapMotionConnected;
}

/*
 *
 */
bool Quadcopter::isArduinoConnected() {
    return mArduinoConnected;
}

/*
 *
 */
PIDcontroller& Quadcopter::getPIDcontroller() {
    return mPIDcontroller;
}

/*
 *
 */
TrackerFinder& Quadcopter::getTrackerFinder() {
    return mTrackerFinder;
}


