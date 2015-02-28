/*
 *  Class - GUI
 *  Author - Alex Morgan
 */

#include "GUI.h"

/*
 *  Constructor
 */
GUI::GUI()  {
    // For use in FPS calulations
    tPrev = high_resolution_clock::now();
}

/*
 * Update the data available to the GUI Class.
 */
void GUI::updateData( Quadcopter& pQuadcopter, gl::TextureFontRef& pTextureFont ) {
    mQuadcopter = pQuadcopter;
    
    // Get the new Quadcopter Positions
    m2DPositions = mQuadcopter.get2DPositions();
    m2DCenters = mQuadcopter.get2DCenters();
    m3DPositions = mQuadcopter.get3DPositions();
    m3DCenter = mQuadcopter.get3DCenter();
    
    // Get the Raw IR images
    mProcessedImageTex = mQuadcopter.getCameraImages();
    
    // Get the Fonts
    mTextureFont = pTextureFont;
}

/*
 * Main Draw method.
 *   Input: int - Mode of GUI (Default: 0)
 */
void GUI::draw( int pMode ) {
    
    // Calculate FPS
    float tDiff = std::chrono::duration_cast<std::chrono::microseconds>( high_resolution_clock::now() - tPrev ).count();
    mFPS = (1000000)/(tDiff);
    tPrev = high_resolution_clock::now();
    
    // Call Appropriate Draw methods
    switch ( pMode) {
        case 0: // Default
            // Draw 3D perspective view
            draw3D( 0, 0, 2*mProcessedImageTex[0].getHeight() );
            // Draw Raw IR cameras view
            drawCameras( 2*mProcessedImageTex[0].getHeight(), 0 );
            // Draw General Info view
            drawGeneral( 0, 2*mProcessedImageTex[0].getHeight(), mProcessedImageTex[0].getWidth()+2*mProcessedImageTex[0].getHeight(), 200 );
            break;
        case 1: // Custom
            break;
    }
    
}

/*
 *  Draws the two (Left & Right) raw IR camera image streams. The view is organised as the Right and Left cameras on Top and Bottom respectivley.
 *  The tracked dots are overlayed onto the image. This also includes an simulated quadcopter with direction.
 *  Input : int pX & pY - Top left index of View
 */
void GUI::drawCameras( int pX, int pY ) {
    
    // Unset Circles Colour
    gl::color(1,1,1);
    
    // Width and Height for convinient use
    int tFullW = mProcessedImageTex[0].getWidth();
    int tFullH = mProcessedImageTex[0].getHeight();
    
    // Draw Images within specified bounds
    gl::draw( mProcessedImageTex[0], Area(Vec2f(pX,pY), Vec2f(pX+tFullW,pY+tFullH) ) );
    gl::draw( mProcessedImageTex[1], Area(Vec2f(pX, pY+tFullH), Vec2f(pX+tFullW, pY+2*tFullH)) );
    
    // If the Quadcopter was detected in the last frame
    if ( mQuadcopter.getFrameSuccess() ) {
        
        // Connected Crosses
        if ( m2DPositions.size() == 4 ) {
            // Set Cross Colour
            gl::color(1, 1, 0);
            
            // Cam 1
            gl::drawLine( Vec2f( pX+m2DPositions[0][0][0], pY+m2DPositions[0][0][1] ), Vec2f( pX+m2DPositions[2][0][0], pY+m2DPositions[2][0][1] ) );
            gl::drawLine( Vec2f( pX+m2DPositions[1][0][0], pY+m2DPositions[1][0][1] ), Vec2f( pX+m2DPositions[3][0][0], pY+m2DPositions[3][0][1] ) );
            // Cam 2
            gl::drawLine( Vec2f( pX+m2DPositions[0][1][0], pY+m2DPositions[0][1][1]+tFullH ), Vec2f( pX+m2DPositions[2][1][0], pY+m2DPositions[2][1][1]+tFullH ) );
            gl::drawLine( Vec2f( pX+m2DPositions[1][1][0], pY+m2DPositions[1][1][1]+tFullH ), Vec2f( pX+m2DPositions[3][1][0], pY+m2DPositions[3][1][1]+tFullH ) );
        }
        
        // Direction Arrows
        drawArrow( m2DCenters[0], m2DCenters[2][0], 30.0, pX, pY, false );
        drawArrow( m2DCenters[1], m2DCenters[2][1], 30.0, pX, pY+tFullH, false );
        
        // Set Circles Colour
        gl::color(1, 0, 0);
        
        // Draw Tracked Dots
        for ( int tCam = 0; tCam < 2; tCam++ ) {
            for ( int tDot = 0; tDot < m2DPositions.size(); tDot++ ) {
                // Cycle Colour
                switch ( tDot%4 ) {
                    case 0:
                        gl::color(1, 0, 0);
                        break;
                    case 1:
                        gl::color(0, 1, 0);
                        break;
                    case 2:
                        gl::color(0, 0, 1);
                        break;
                    case 3:
                        gl::color(1, 0, 1);
                        break;
                    default:
                        gl::color(1, 1, 1);
                        break;
                }
                // Draw Circles over Tracked Dots
                switch ( tCam ) {
                    case 0:
                        gl::drawSolidCircle( Vec2f( pX+m2DPositions[tDot][tCam][0], pY+m2DPositions[tDot][tCam][1] ), 3.0 );
                        break;
                    case 1:
                        gl::drawSolidCircle( Vec2f( pX+m2DPositions[tDot][tCam][0], pY+m2DPositions[tDot][tCam][1]+tFullH ), 3.0 );
                        break;
                }
            }
        }
    }
    
    // Draw Labels Text
    gl::color(1, 1, 1); // White
    mTextureFont->drawString("Right Camera",  Vec2f( pX+10.0, pY+25.0 ) );
    mTextureFont->drawString("Left Camera",  Vec2f( pX+10.0, pY+25.0 + tFullH ) );
    
    // Draw Borders
    gl::color( 1,1,1 );
    // Horizontal Top
    gl::drawLine( Vec2f( pX, pY ), Vec2f( mProcessedImageTex[0].getWidth()+pX,pY ) );
    // Horiztonal Center
    gl::drawLine( Vec2f( pX, mProcessedImageTex[0].getHeight()+pY ), Vec2f( mProcessedImageTex[0].getWidth()+pX, mProcessedImageTex[0].getHeight()+pY ) );
    // Horizontal Bottom
    gl::drawLine( Vec2f( pX, 2*mProcessedImageTex[0].getHeight()+pY ), Vec2f( mProcessedImageTex[0].getWidth()+pX, 2*mProcessedImageTex[0].getHeight()+pY ) );
    // Vertical Left
    gl::drawLine( Vec2f( pX, pY ), Vec2f( pX, 2*mProcessedImageTex[0].getHeight()+pY ) );
    // Vertical Right
    gl::drawLine( Vec2f( mProcessedImageTex[0].getWidth()+pX, pY ), Vec2f( mProcessedImageTex[0].getWidth()+pX, 2*mProcessedImageTex[0].getHeight()+pY ) );
     
    // Unset Circles Colour
    gl::color(1,1,1);
}


/*
 *  Draw the 3D perspective view. This consists of the interpreted 3D position from the 2D positions. A top down view is displayed with the height axis
 *  displayed as bar on the side.
 *      Input: int pX, pY & pW - The top left index of the view and the width (must be square)
 */
void GUI::draw3D( int pX, int pY, int pW ) {
    
    // Clear Last Screen
    gl::color( 0,0,0 );
    gl::drawSolidRect( Area( Vec2f( pX , pY ), Vec2f( pX+pW, pY+pW ) ) );
    
    // Draw Cross for Origin of View
    gl::color( 1,1,1 );
    int tCrossLen =  100;
    gl::drawLine( Vec2f( pX+(0.5*pW), pY+(0.5*pW) - tCrossLen ), Vec2f( pX+(0.5*pW), pY+(0.5*pW) + tCrossLen ) );
    gl::drawLine( Vec2f( pX+(0.5*pW) - tCrossLen, pY+(0.5*pW) ), Vec2f( pX+(0.5*pW) + tCrossLen, pY+(0.5*pW) ) );
    
    // Draw Height Bar
    gl::color( 1,1,1 );
    int tGapT = 50;
    int tGapS = 30;
    int tEnd = 5;
    // Vertical Bar
    gl::drawLine( Vec2f( pX+pW-tGapS, pY+tGapT ), Vec2f( pX+pW-tGapS, pY+pW-tGapT ) );
    // Top End
    gl::drawLine( Vec2f( pX+pW-tGapS-tEnd, pY+tGapT ), Vec2f( pX+pW-tGapS+tEnd, pY+tGapT ) );
    // Bottom End
    gl::drawLine( Vec2f( pX+pW-tGapS-tEnd, pY+pW-tGapT ), Vec2f( pX+pW-tGapS+tEnd, pY+pW-tGapT ) );
    
    // If the Quadcopter was detected in the last frame
    if ( mQuadcopter.getFrameSuccess() ) {
        
        // Draw 3D Dots
        // Damp the displacement of the points (expands the X-Y plane)
        float tDamp = 0.6;
        for ( int tDot = 0; tDot < m3DPositions.size(); tDot++ ) {
            // Cycle Colours
            switch ( tDot%4 ) {
                case 0:
                    gl::color(1, 0, 0); // Red
                    break;
                case 1:
                    gl::color(0, 1, 0); // Green
                    break;
                case 2:
                    gl::color(0, 0, 1); // Blue
                    break;
                case 3:
                    gl::color(1, 0, 1); // Cyan
                    break;
                default:
                    gl::color(1, 1, 1); // White
                    break;
            }
            // Draw circles for the X-Y plane coordinates
            gl::drawSolidCircle( Vec2f( pX+(0.5*pW)+(m3DPositions[tDot][0]*tDamp), pY+(0.5*pW)+(m3DPositions[tDot][2]*tDamp) ), 3.0 );
            
        }
        
        // Draw Cross for Origin of Quadcopter
        drawCross( vector<float> { m3DCenter[0]*tDamp, m3DCenter[2]*tDamp }, pX+(0.5*pW), pY+(0.5*pW) );
        
        // Draw Arrow for Direction
        drawArrow( vector<float> { m3DCenter[0]*tDamp, m3DCenter[2]*tDamp }, m3DCenter[3], 30, pX+(0.5*pW), pY+(0.5*pW), false );
        
        // Draw Height Indicator
        gl::color(1, 1, 0); // Yellow
        int tMid = 8;
        int tHeight = (m3DCenter[1]/1000) * (pW-(2*tGapT));
        gl::drawLine( Vec2f( pX+pW-tGapS-tMid, pY+pW-tGapT-tHeight ), Vec2f( pX+pW-tGapS+tMid, pY+pW-tGapT-tHeight ) );
        
        // Draw PID Reponses
        vector<float> mPIDResponse = mQuadcopter.getPIDcontroller().caclulateControlSignal( false );
    
        // Draw X-Y plane PID response
        float tAng = findAngle( vector<float> { mPIDResponse[3], mPIDResponse[2] }, vector<float> { 0.0f, 0.0f } );
        float tStrength = sqrt( pow( mPIDResponse[3], 2.0) + pow( mPIDResponse[2], 2.0) ) * 50.0;
        drawArrow( vector<float> { m3DCenter[0]*tDamp, m3DCenter[2]*tDamp }, tAng, tStrength, pX+(0.5*pW), pY+(0.5*pW), true );
    
        // Draw Z axis PID response
        tStrength = mPIDResponse[0] * 60.0;
        gl::drawLine( Vec2f( pX+pW-tGapS-tMid, pY+pW-tGapT-tHeight ), Vec2f( pX+pW-tGapS-tMid, pY+pW-tGapT-tHeight-tStrength ) );
        gl::drawLine( Vec2f( pX+pW-tGapS+tMid, pY+pW-tGapT-tHeight ), Vec2f( pX+pW-tGapS+tMid, pY+pW-tGapT-tHeight-tStrength ) );
        
    }
        
    // Draw Label Text
    gl::color( 1,1,1 ); // White
    mTextureFont->drawString("3D",  Vec2f( pX+10.0, pY+25.0 ) );
    
    // Draw Borders
    gl::color( 1,1,1 ); // White
    // Horizontal Top
    gl::drawLine( Vec2f( pX, pY ), Vec2f( pX+pW,pY ) );
    // Horizontal Bottom
    gl::drawLine( Vec2f( pX, pY+pW ), Vec2f( pX+pW, pY+pW ) );
    // Vertical Left
    gl::drawLine( Vec2f( pX, pY ), Vec2f( pX, pY+pW ) );
    // Vertical Right
    gl::drawLine( Vec2f( pX+pW, pY ), Vec2f( pX+pW, pY+pW ) );
    
    // Unset Circles Colour
    gl::color(1,1,1);
}

/*
 *  Draw thw General Info view. This includes information such as the FPS, whether LeapMotion adn Arduino are connected. This will include
 *  more information in the future including data about the PID control processes.
 *      Input: int pX, pY, pW & pH - Top left index of the View, as well the width and height
 */
void GUI::drawGeneral( int pX, int pY, int pW, int pH ) {
    
    // Clear Last Screen
    gl::color( 0,0,0 );
    gl::drawSolidRect( Area( Vec2f( pX , pY ), Vec2f( pX+pW, pY+pH ) ) );
    
    // 1st Column
    gl::color( 1,1,1 ); // White
    int tColW = 40;
    int tColH = 60;
    int tColSepW = 150;
    int tColSepH = 20;
    
    // FPS Status
    ostringstream ss;
    ss << mFPS;
    mTextureFont->drawString("FPS:",  Vec2f( pX+tColW+(0*tColSepW), pY+tColH+(0*tColSepH) ) );
    mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(1*tColSepW), pY+tColH+(0*tColSepH) ) );
    ss.str(std::string());
    ss.clear();
    
    // LeapMotion Status
    gl::color( 1,1,1 ); // White
    bool tLeapConnected = mQuadcopter.isLeapMotionConnected();
    if ( tLeapConnected ) {
        mTextureFont->drawString("LeapMotion:",  Vec2f( pX+tColW+(0*tColSepW), pY+tColH+(1*tColSepH) ) );
        gl::color( 0,1,0 ); // Green
        mTextureFont->drawString("Connected",  Vec2f( pX+tColW+(1*tColSepW), pY+tColH+(1*tColSepH) ) );
    } else {
        mTextureFont->drawString("LeapMotion:",  Vec2f( pX+tColW+(0*tColSepW), pY+tColH+(1*tColSepH) ) );
        gl::color( 1,0,0 ); // Red
        mTextureFont->drawString("Not Connected",  Vec2f( pX+tColW+(1*tColSepW), pY+tColH+(1*tColSepH) ) );
    }
    
    // Arduino Status
    gl::color( 1,1,1 ); // White
    bool tArduinoConnected = mQuadcopter.isArduinoConnected();
    if ( tArduinoConnected ) {
        mTextureFont->drawString("Arduino:",  Vec2f( pX+tColW+(0*tColSepW), pY+tColH+(2*tColSepH) ) );
        gl::color( 0,1,0 ); // Green
        mTextureFont->drawString("Connected",  Vec2f( pX+tColW+(1*tColSepW), pY+tColH+(2*tColSepH) ) );
    } else {
        mTextureFont->drawString("Arduino:",  Vec2f( pX+tColW+(0*tColSepW), pY+tColH+(2*tColSepH) ) );
        gl::color( 1,0,0 ); // Red
        mTextureFont->drawString("Not Connected",  Vec2f( pX+tColW+(1*tColSepW), pY+tColH+(2*tColSepH) ) );
    }
    
    // Draw PID Info
    gl::color( 1,1,1 ); // White
    // Update Column/Row separaters
    tColW = 400;
    tColH = 60;
    tColSepW = 120;
    tColSepH = 20;
    ss.precision(3);
    ss.setf(std::ios::fixed);
    
    // Find if Frame was successful - ie a Quad was found
    bool tSuccess = mQuadcopter.getTrackerFinder().lastFrameSuccess();
    
    // Get Control Signals
    vector< vector<float> > tComps = vector< vector<float> > ( 4, vector<float>( 4, 0.0 ) );
    if ( tSuccess ) {
        tComps = mQuadcopter.getPIDcontroller().getControlSignalComponents();
    }
    
    mTextureFont->drawString("DoF",  Vec2f( pX+tColW+(0*tColSepW), pY+tColH+(0*tColSepH) ) );
    mTextureFont->drawString("Throttle",  Vec2f( pX+tColW+(0*tColSepW), pY+tColH+(1*tColSepH) ) );
    mTextureFont->drawString("Rudder",  Vec2f( pX+tColW+(0*tColSepW), pY+tColH+(2*tColSepH) ) );
    mTextureFont->drawString("Elevator",  Vec2f( pX+tColW+(0*tColSepW), pY+tColH+(3*tColSepH) ) );
    mTextureFont->drawString("Aileron",  Vec2f( pX+tColW+(0*tColSepW), pY+tColH+(4*tColSepH) ) );
    
    mTextureFont->drawString("P",  Vec2f( pX+tColW+(1*tColSepW), pY+tColH+(0*tColSepH) ) );
    if ( tSuccess ) {
        ss << tComps[0][0];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(1*tColSepW), pY+tColH+(1*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
        ss << tComps[1][0];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(1*tColSepW), pY+tColH+(2*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
        ss << tComps[2][0];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(1*tColSepW), pY+tColH+(3*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
        ss << tComps[3][0];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(1*tColSepW), pY+tColH+(4*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
    }
    
    
    mTextureFont->drawString("I",  Vec2f( pX+tColW+(2*tColSepW), pY+tColH+(0*tColSepH) ) );
    if ( tSuccess ) {
        ss << tComps[0][1];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(2*tColSepW), pY+tColH+(1*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
        ss << tComps[1][1];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(2*tColSepW), pY+tColH+(2*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
        ss << tComps[2][1];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(2*tColSepW), pY+tColH+(3*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
        ss << tComps[3][1];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(2*tColSepW), pY+tColH+(4*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
    }
    
    mTextureFont->drawString("D",  Vec2f( pX+tColW+(3*tColSepW), pY+tColH+(0*tColSepH) ) );
    if ( tSuccess ) {
        ss << tComps[0][2];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(3*tColSepW), pY+tColH+(1*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
        ss << tComps[1][2];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(3*tColSepW), pY+tColH+(2*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
        ss << tComps[2][2];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(3*tColSepW), pY+tColH+(3*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
        ss << tComps[3][2];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(3*tColSepW), pY+tColH+(4*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
    }
        
    mTextureFont->drawString("Total",  Vec2f( pX+tColW+(4*tColSepW), pY+tColH+(0*tColSepH) ) );
    if ( tSuccess ) {
        ss << tComps[0][3];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(4*tColSepW), pY+tColH+(1*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
        ss << tComps[1][3];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(4*tColSepW), pY+tColH+(2*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
        ss << tComps[2][3];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(4*tColSepW), pY+tColH+(3*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
        ss << tComps[3][3];
        mTextureFont->drawString(ss.str(),  Vec2f( pX+tColW+(4*tColSepW), pY+tColH+(4*tColSepH) ) );
        ss.str(std::string());
        ss.clear();
    }
    
    
    // Draw Label Text
    gl::color( 1,1,1 ); // White
    mTextureFont->drawString("Info",  Vec2f( pX+10.0, pY+25.0 ) );
    
    // Draw Borders
    gl::color( 1,1,1 );
    // Horizontal Top
    gl::drawLine( Vec2f( pX, pY ), Vec2f( pX+pW,pY ) );
    // Horizontal Bottom
    gl::drawLine( Vec2f( pX, pY+pH ), Vec2f( pX+pW, pY+pH ) );
    // Vertical Left
    gl::drawLine( Vec2f( pX, pY ), Vec2f( pX, pY+pH ) );
    // Vertical Right
    gl::drawLine( Vec2f( pX+pW, pY ), Vec2f( pX+pW, pY+pH ) );
    
    // Unset Circles Colour
    gl::color(1,1,1);
}

/*
 *  Draw an Arrow.
 */
void GUI::drawArrow( vector<float> pStart, float pDir, float pLength, int pX, int pY, bool pBody ) {
    
    // Set Color
    gl::color(1, 1, 0); // Yellow
    
    // Convert Direction into Radians
    float mDir = pDir * ( PI / 180.0 );
    
    // End Point
    vector<float> tEnd = { pStart[0]+( pLength*sin(mDir) ), pStart[1]+( pLength*cos(mDir) ) };
    
    // Draw Body
    if (pBody) {
        gl::drawLine( Vec2f( pX+pStart[0], pY+pStart[1] ), Vec2f( pX+tEnd[0], pY+tEnd[1] ) );
    }
    
    // Draw Head
    float tHeadLen = 10.0;
    float tHeadAngle = 135.0 * ( PI / 180.0 );
    gl::drawLine( Vec2f( tEnd[0]+pX, tEnd[1]+pY ), Vec2f( tEnd[0] + tHeadLen*sin(mDir+tHeadAngle)+pX, tEnd[1] + tHeadLen*cos(mDir+tHeadAngle)+pY ) );
    gl::drawLine( Vec2f( tEnd[0]+pX, tEnd[1]+pY ), Vec2f( tEnd[0] + tHeadLen*sin(mDir-tHeadAngle)+pX, tEnd[1] + tHeadLen*cos(mDir-tHeadAngle)+pY ) );
    
}

/*
 *  Draw a Cross
 */
void GUI::drawCross( vector<float> pPos, int pX, int pY ) {
    
    // Set Color
    gl::color(1, 1, 0); // Yellow
    
    // Lengths
    float tLen = 5;
    
    // Draw Body
    gl::drawLine( Vec2f( pPos[0]+pX, pPos[1]+pY-tLen ), Vec2f( pPos[0]+pX, pPos[1]+pY+tLen ) );
    gl::drawLine( Vec2f( pPos[0]+pX-tLen, pPos[1]+pY ), Vec2f( pPos[0]+pX+tLen, pPos[1]+pY ) );
    
}

/*
 *  Given an origin and Point, finds the angle to it in degrees.
 *  Input  : Origin and Point
 *  Output : Angle from Origin to Point
 */
float GUI::findAngle( vector<float> pOrig, vector<float> pPoint ) {
    
    // Find Deltas
    float dX = pPoint[0] - pOrig[0];
    float dY = pPoint[1] - pOrig[1];
    
    // Find Angle in Radians
    float mAngleRad = atan( dX / dY );
    
    // Correct for Quadrants
    if ( dX < 0 && dY < 0 ) {
        mAngleRad += PI;
    } else if ( dX < 0 && dY > 0 ) {
        mAngleRad +=  2*PI;
    } else if ( dX > 0 && dY < 0 ) {
        mAngleRad +=  PI;
    }
    
    // Convert to Degrees
    float mAngleDeg = mAngleRad * (180 / PI);
    
    return mAngleDeg;
}
