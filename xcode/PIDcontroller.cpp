/*
 *  Class - PIDcontroller
 *  Author - Alex Morgan
 */

#include "PIDcontroller.h"

/*
 *  Constructor
 */
PIDcontroller::PIDcontroller() {
    
    // Specific PID weights
    mPID = {
    //     P      I      D
        { -0.5f,  0.0f,  0.0f }, // Throttle
        {  0.0f,  0.0f,  0.0f }, // Rudder
        {  0.5f,  0.5f,  0.0f }, // Elevator
        {  0.5f,  0.5f,  0.0f }  // Aileron
    }; // I = 0.2f Ele and Ail
    
    // Apply Scalings
    vector<float> mPIDx { 0.01, 0.1, 1000000.0 };
    for ( int i=0; i<4; i++ ) {
        mPID[i][0] *= mPIDx[0];
        mPID[i][1] *= mPIDx[1];
        mPID[i][2] *= mPIDx[2];
    }
    
    // Init Time
    mPrev = high_resolution_clock::now();
    mDeltaTime = 0.0;
    mThresholdTimeout = 1.0; // Seconds
    
    // Set Reference position to Zero
    //     Up-Down, Rotation, Forward-Back, Left Right
    mReferencePosition      = { 0.0, 0.0, 0.0, 0.0 };
    
    // Set Current position to Zero
    mCurrentPosition        = { 0.0, 0.0, 0.0, 0.0 };
    
    // Init Errors to Zero
    mErrorInPosition        = { 0.0, 0.0, 0.0, 0.0 };
    mRunningErrorInPosition = { 0.0, 0.0, 0.0, 0.0 };
    mVelocity               = { 0.0, 0.0, 0.0, 0.0 };
    
    // Init Helper Variables for Running Error in Position - ie Integral
    mRunningErrorInPositionWindowSize = 60; // About 3 Seconds at 20 FPS
    mRunningErrorInPositionWindowIndex = 0;
    mRunningErrorInPositionWindow = vector< vector<float> > ( 4, vector<float>( mRunningErrorInPositionWindowSize, 0.0 ) );
    
    // Init control signal components
    mControlSignalComponents = vector< vector<float> > ( 4, vector<float>( 4, 0.0 ) );
    
    // Set first frame to true
    mFirstFrame = true;
    
    // Set Limits for Controls
    mLimits = { vector<float> {  0.0, 1.0 },
                vector<float> { -1.0, 1.0 },
                vector<float> { -1.0, 1.0 },
                vector<float> { -1.0, 1.0 } };
    
}

/*
 *
 */
void PIDcontroller::update( vector<float> pPos ) {
    
    // Update all in turn - Velocity must be before CurrentPosition
    updateTime();
    updateVelocity( pPos );
    updateCurrentPosition( pPos );
    updateErrorInPosition();
    updateRunningErrorInPosition();
    
    // End of First Frame
    if ( mFirstFrame )
        mFirstFrame = false;
    
}

/*
 *  These are the PID controller equations!
 */
vector<float> PIDcontroller::caclulateControlSignal( bool pPrint ) {
    vector<float> tVec =
    {
          ( mPID[0][0] * mErrorInPosition[0]         /* Proportional Term */ ) // X - Up Down
        + ( mPID[0][1] * mRunningErrorInPosition[0]  /* Integral Term     */ )
        + ( mPID[0][2] * mVelocity[0]                /* Differential Term */ ),

          ( mPID[1][0] * mErrorInPosition[1]         /* Proportional Term */ ) // R - Clockwise Anti-Clockwise
        + ( mPID[1][1] * mRunningErrorInPosition[1]  /* Integral Term     */ )
        + ( mPID[1][2] * mVelocity[1]                /* Differential Term */ ),
        
          ( mPID[2][0] * mErrorInPosition[2]         /* Proportional Term */ ) // Y - Forward Back
        + ( mPID[2][1] * mRunningErrorInPosition[2]  /* Integral Term     */ )
        + ( mPID[2][2] * mVelocity[2]                /* Differential Term */ ),
                       
          ( mPID[3][0] * mErrorInPosition[3]         /* Proportional Term */ ) // Z - Left Right
        + ( mPID[3][1] * mRunningErrorInPosition[3]  /* Integral Term     */ )
        + ( mPID[3][2] * mVelocity[3]                /* Differential Term */ )
    };
    
    // Sanatise Controls - make sure they are within the limits
    tVec = sanatise( tVec );
    
    // Save components - For GUI
    mControlSignalComponents[0] = { mPID[0][0] * mErrorInPosition[0],
                                    mPID[0][1] * mRunningErrorInPosition[0],
                                    mPID[0][2] * mVelocity[0],
                                    tVec[0] };
    
    mControlSignalComponents[1] = { mPID[1][0] * mErrorInPosition[1],
                                    mPID[1][1] * mRunningErrorInPosition[1],
                                    mPID[1][2] * mVelocity[1],
                                    tVec[1] };
    
    mControlSignalComponents[2] = { mPID[2][0] * mErrorInPosition[2],
                                    mPID[2][1] * mRunningErrorInPosition[2],
                                    mPID[2][2] * mVelocity[2],
                                    tVec[2] };
    
    mControlSignalComponents[3] = { mPID[3][0] * mErrorInPosition[3],
                                    mPID[3][1] * mRunningErrorInPosition[3],
                                    mPID[3][2] * mVelocity[3],
                                    tVec[3] };
    
    // Print To Console
    if ( pPrint ) {
        console() << "------------------------------------------------------" << endl;
        printf( "%10s\t%10s\t%10s\t%10s\t%10s\n", "DoF", "P", "I", "D", "Total" );
        printf( "%10s\t%10f\t%10f\t%10f\t%10f\n", "Throttle",
               mControlSignalComponents[0][0],
               mControlSignalComponents[0][1],
               mControlSignalComponents[0][2],
               mControlSignalComponents[0][3]);
        printf( "%10s\t%10f\t%10f\t%10f\t%10f\n", "Rudder",
               mControlSignalComponents[1][0],
               mControlSignalComponents[1][1],
               mControlSignalComponents[1][2],
               mControlSignalComponents[1][3]);
        printf( "%10s\t%10f\t%10f\t%10f\t%10f\n", "Elevator",
               mControlSignalComponents[2][0],
               mControlSignalComponents[2][1],
               mControlSignalComponents[2][2],
               mControlSignalComponents[2][3]);
        printf( "%10s\t%10f\t%10f\t%10f\t%10f\n", "Aileron",
               mControlSignalComponents[3][0],
               mControlSignalComponents[3][1],
               mControlSignalComponents[3][2],
               mControlSignalComponents[3][3]);
        console() << "------------------------------------------------------" << endl;
    }

    return tVec;
}

/*
 *
 */
void PIDcontroller::updateTime() {
    
    mDeltaTime = std::chrono::duration_cast<std::chrono::microseconds>( high_resolution_clock::now() - mPrev ).count() / 1000000.0;
    mPrev = high_resolution_clock::now();
    
    // If Delta Time is above a thresold, then Zero all Time dependent Variables
    if ( mDeltaTime > mThresholdTimeout ) {
        // Init Errors to Zero
        mErrorInPosition        = { 0.0, 0.0, 0.0, 0.0 };
        mRunningErrorInPosition = { 0.0, 0.0, 0.0, 0.0 };
        mVelocity               = { 0.0, 0.0, 0.0, 0.0 };
        
        // Reset Helper Variables
        mRunningErrorInPositionWindow = vector< vector<float> > ( 4, vector<float>( mRunningErrorInPositionWindowSize, 0.0 ) );
        mRunningErrorInPositionWindowIndex = 0;
        
        // Reset Delta Time
        mDeltaTime = 0.0;
        
        // Print Status
        if ( VERBOSE_ERROR ) console() << "Timeout Threshold reached! Time Dependent Variables have been Zeroed." << endl;
    }
}

/*
 *
 */
vector<float> PIDcontroller::getReferencePosition() {
    return mReferencePosition;
}

/*
 *
 */
void PIDcontroller::updateReferencePosition( vector<float> pPos ) {
    mReferencePosition = pPos;
}

/*
 *
 */
vector<float> PIDcontroller::getCurrentPosition() {
    return mCurrentPosition;
}

/*
 *
 */
void PIDcontroller::updateCurrentPosition( vector<float> pPos ) {
    mCurrentPosition = pPos;
}

/*
 *
 */
vector<float> PIDcontroller::getErrorInPosition() {
    return mErrorInPosition;
}

/*
 *
 */
void PIDcontroller::updateErrorInPosition() {

    // Only execute if update function has been called at least once
    if ( !mFirstFrame ) {
        vector<float> tVec = {
            mCurrentPosition[0] - mReferencePosition[0],
            mCurrentPosition[1] - mReferencePosition[1],
            mCurrentPosition[2] - mReferencePosition[2],
            mCurrentPosition[3] - mReferencePosition[3]
        };
        
        mErrorInPosition = tVec;
    }
}

/*
 *
 */
vector<float> PIDcontroller::getRunningErrorInPosition() {
    return mRunningErrorInPosition;
}

/*
 *
 */
void PIDcontroller::updateRunningErrorInPosition() {
    
    // Only execute if update function has been called at least once
    if ( !mFirstFrame ) {
    
        // Loop back such that to only have last N contributions
        int tIndex = mRunningErrorInPositionWindowIndex % mRunningErrorInPositionWindowSize;
        
        // Add Area under timestep to Running Vectors
        mRunningErrorInPositionWindow[0][tIndex] = ( mErrorInPosition[0] * mDeltaTime );
        mRunningErrorInPositionWindow[1][tIndex] = ( mErrorInPosition[1] * mDeltaTime );
        mRunningErrorInPositionWindow[2][tIndex] = ( mErrorInPosition[2] * mDeltaTime );
        mRunningErrorInPositionWindow[3][tIndex] = ( mErrorInPosition[3] * mDeltaTime );
        
        // Increment the Window Index
        mRunningErrorInPositionWindowIndex ++;
        
        // Sum up the Running Vectors to find mRunningErrorInPosition
        vector<float> tVec = { 0.0f, 0.0f, 0.0f, 0.0f };
        for ( int i = 0; i < mRunningErrorInPositionWindowSize; i++ ) {
            for ( int j = 0; j < 4; j++ ) tVec[j] += mRunningErrorInPositionWindow[j][i];
        }
        
        // Divide by number of frames to make the result time constant independent
        for ( int i = 0; i < 4; i++ ) tVec[i] /= mRunningErrorInPositionWindowSize;

        // Update Running Error in Position
        mRunningErrorInPosition = tVec;
    }
}

/*
 *
 */
vector<float> PIDcontroller::getVelocity() {
    return mVelocity;
}

/*
 *
 */
void PIDcontroller::updateVelocity( vector<float> pPos ) {
    
    // Only execute if update function has been called at least once
    if ( !mFirstFrame ) {
        vector<float> tVec = {
            ( pPos[0] - mCurrentPosition[0] ) / mDeltaTime,
            ( pPos[1] - mCurrentPosition[1] ) / mDeltaTime,
            ( pPos[2] - mCurrentPosition[2] ) / mDeltaTime,
            ( pPos[3] - mCurrentPosition[3] ) / mDeltaTime
        };
        
        mVelocity = tVec;
    }
}

/*
 *
 */
vector< vector<float> > PIDcontroller::getPID() {
    return mPID;
}

/*
 *
 */
void PIDcontroller::updatePID( vector< vector<float> > pPID ) {
    mPID = pPID;
}

/*
 *
 */
vector<float> PIDcontroller::sanatise( vector<float> pVec ) {
    
    vector<float> tVec = pVec;
    
    // Sanitise Controls
    for ( int i=0; i < tVec.size(); i++ ) {
        if ( tVec[i] < mLimits[i][0] )
            tVec[i] = mLimits[i][0];
        if ( tVec[i] > mLimits[i][1] )
            tVec[i] = mLimits[i][1];
    }
    
    return tVec;
}

/*
 *
 */
vector< vector<float> > PIDcontroller::getControlSignalComponents() {
    return mControlSignalComponents;
}

