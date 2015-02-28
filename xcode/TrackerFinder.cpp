/*
 *  Class - TrackerFinder
 *  Author - Alex Morgan
 */

#include "TrackerFinder.h"

/*
 *  Blank Constructor
 */
TrackerFinder::TrackerFinder() {

}

/*
 *  Constructor
 */
TrackerFinder::TrackerFinder(int pDots) {
    mNumTrack = pDots;
    mLastFrameSuccess = false;
    
    // Init to Zeros
    mDotsTrio = vector< vector< vector<float> > > ( mNumTrack, vector< vector<float> > ( 3, vector<float>( 3, 0.0 ) ) );
    mOrigins = vector< vector<float> > ( 3, vector<float>( 3, 0.0 ) );
    mDirections = vector<float> ( 3, 0.0 );
}

/*
 *  Given Left and Right Camera Surfaces, uses available detection methods to return a vector< vector< float > >
 *  containing the 3D real position and 2D Camera position of the dots.
 *  Input  : Distorted Surfaces for Camera 0 & 1
 *  Output : 2D Distorted Surface positions and 3D Real Space positions 
 *              - Vector< [ [X(2D)0, Y(2D)0, 0], [X(2D)1, Y(2D)1, 0], [X(3D), Y(3D), Z(3D)] ] >
 */
vector< vector< vector<float> > > TrackerFinder::findDots( vector<Image> pImages, vector<Surface> pSurfaces ) {
    
    // Reset Frame Success
    mLastFrameSuccess = false;
    
    // Reset Data
    mDotsTrio = vector< vector< vector<float> > > ( mNumTrack, vector< vector<float> > ( 3, vector<float>( 3, 0.0 ) ) );
    mOrigins = vector< vector<float> > ( 3, vector<float>( 3, 0.0 ) );
    mDirections = vector<float> ( 3, 0.0 );
    
    // Find Keypoints in each Camera Surface - Detection method specific *
    //      Make sure there are only 'num_track' number of Keypoints
    vector< cv::KeyPoint > keypoints[2];
    
    keypoints[0] = findSimpleBlobs( pSurfaces[0].clone() , 0 );
    keypoints[1] = findSimpleBlobs( pSurfaces[1].clone() , 1 );
    
    // Assumption : Will only accept detection if 4 blobs found in each.
    if ( keypoints[0].size() != mNumTrack || keypoints[1].size() != mNumTrack ) {
        // Failed
        if ( VERBOSE_ERROR ) console() << "*** Failed! Did not find expected number of Dots in both Images! ***" << endl;
        if ( VERBOSE_ERROR )console() << "    Size : " << keypoints[0].size() << endl;
        if ( VERBOSE_ERROR )console() << "    Size : " << keypoints[1].size() << endl;
        
        // Return with empty Dots!
        return vector< vector< vector<float> > >( 0, vector< vector<float> >( 3, vector<float>(3) ) );
    } else {
        // Set Frame Success
        mLastFrameSuccess = true;
    }
    
    // Convert to Vector< Vector< float > > for Dot normalisation - Detection method specific *
    mDotsTrio = vector< vector< vector<float> > >( mNumTrack, vector< vector<float> >( 3, vector<float>(3) ) );
    
    for ( int tCam = 0; tCam < 2; tCam++ ) {
        for ( int tDot = 0; tDot < keypoints[0].size(); tDot++ ) {
            mDotsTrio[tDot][tCam][0] = keypoints[tCam][tDot].pt.x;
            mDotsTrio[tDot][tCam][1] = keypoints[tCam][tDot].pt.y;
        }
    }
    
    // Find Origins & Directions & Predicted Dots. For 2D
    postProcessing2D( pImages );
    
    // Pair Dots from each Camera Surface. Reorders the Dots to pair up
    pairDots( pImages );
    
    // Find 3D Real Position
    find3DPositions( pImages );
    
    // Find Origins & Directions For 3D
    postProcessing3D( pImages );


    // Return Dots
    return mDotsTrio;
}

/*
 *  Simple Blob Detector - Preferred Method
 *  Input  : Surface, and the Camera to which is originated
 *  Output : Keypoints found by the SimpleBlobDetector Algorithm
 */
vector< cv::KeyPoint > TrackerFinder::findSimpleBlobs( Surface pSurface, int pCamera ) {
    
    // Convert Surface to OpenCV Mat
    cv::Mat inputMat( toOcv( Channel( pSurface ) ) );
    
    // set up the parameters (check the defaults in opencv's code in blobdetector.cpp)    
    cv::SimpleBlobDetector::Params params;
    params.thresholdStep = 5.0; //float thresholdStep; Default : 10
    params.minThreshold = 230.0; //float minThreshold; Default : 50 // 230
    params.maxThreshold = 350.0; //float maxThreshold; Default : 220 // 350
    params.minRepeatability = 2; //size_t minRepeatability; Default : 2
    params.minDistBetweenBlobs = 1.0; //float minDistBetweenBlobs; Default : 10
    
    params.filterByColor = false; //bool filterByColor;  Default : true
    params.blobColor = 0; //uchar blobColor; Default : 0
    
    params.filterByArea = true; //bool filterByArea;  Default : true
    params.minArea = 3.0; //float minArea; Default : 25
    params.maxArea = 200.0; //float maxArea; Default : 5000
    
    params.filterByCircularity = false; //bool filterByCircularity;  Default : false
    params.minCircularity = 1.0; //float minCircularity; Default : 0.8f
    params.maxCircularity = 1.0; //float maxCircularity; Default : std::numeric_limits<float>::max();
    
    params.filterByInertia = false; //bool filterByInertia;  Default : true
    params.minInertiaRatio = 1.0; //float minInertiaRatio; Default : 0.1f
    params.maxInertiaRatio = 1.0; //float maxInertiaRatio; Default : std::numeric_limits<float>::max();
    
    params.filterByConvexity = false; //bool filterByConvexity;  Default : true
    params.minCircularity = 1.0; //float minConvexity; Default : 0.95f
    params.maxConvexity = 1.0; //float maxConvexity; Default : std::numeric_limits<float>::max();
    
    // Pre-Processing
        // Medium Blur
    //cv::medianBlur(inputMat, inputMat, 3);
        // Sharpen Image
    cv::Mat inputMatSharp;
    cv::GaussianBlur( inputMat, inputMatSharp, cv::Size(9, 9), 2, 2 );
    cv::addWeighted(inputMat, 1.5, inputMatSharp, -0.5, 0, inputMat);
    
    // set up and create the detector using the parameters
    cv::Ptr<cv::FeatureDetector> blob_detector = new cv::SimpleBlobDetector(params);
    blob_detector->create("SimpleBlob");
    
    // detect!
    vector<cv::KeyPoint> keypoints;
    blob_detector->detect(inputMat, keypoints);
    
    // Summary
    //cout << "Size : " << keypoints.size() << endl;
    //for (int i=0; i<keypoints.size(); i++){
        //float X=keypoints[i].pt.x;
        //float Y=keypoints[i].pt.y;
        //cout << "X : " << X << "  Y : " << Y << endl;
    //}
    
    // Set Current Proccessed Surface
    setProcessedSurface( fromOcv( inputMat ), pCamera );
    
    // Return Keypoints
    return keypoints;
}

/*
 *  Hough Circle Transform - Not Used
 *  Input  : Surface, and the Camera to which is originated
 *  Output : Circles found by the Hough Circle Transform. [X, Y, R]
 */
vector< vector<float> > TrackerFinder::findHoughCircles( Surface pSurface, int pCamera ) {
    
    // Convert Surface to OpenCV Mat
    cv::Mat inputMat( toOcv( Channel( pSurface ) ) );
    
    // Hough Circle Parameters
    vector< cv::Vec3f > circles; // Store results.
    int mMethod = CV_HOUGH_GRADIENT; // Detection Method - Only one available.
    double mDP = 1.0; // Inverse ratio of the accumulator resolution to the image resolution.
    double mMinDist = 10.0; // Minimum distance between detected centers.
    double mParam1 = 100; // Upper threshold for the internal Canny edge detector.
    double mParam2 = 20; // Threshold for center detection.
    int mMinRadius = 1; // Minimum radio to be detected. If unknown, put zero as default.
    int mMaxRadius = 50; // Maximum radius to be detected. If unknown, put zero as default.
    
    // Pre-Processing
        // Medium blur
    //cv::medianBlur(inputMat, inputMat, 5);
        // Threshold
    //cv::threshold(inputMat, inputMat, 50.0, 255, 0);
        // Sharpen Image
    cv::Mat inputMatSharp;
    cv::GaussianBlur( inputMat, inputMatSharp, cv::Size(9, 9), 2, 2 );
    cv::addWeighted(inputMat, 1.5, inputMatSharp, -0.5, 0, inputMat);
    
    // Convert it to gray
    //cv::cvtColor( inputMat, inputMat, CV_BGR2GRAY );
    
    // Hough Circle Transform
    //void HoughCircles(InputArray image, OutputArray circles, int method, double dp, double minDist,
    //                  double param1=100, double param2=100, int minRadius=0, int maxRadius=0 )
    cv::HoughCircles(inputMat, circles, mMethod, mDP, mMinDist, mParam1, mParam2, mMinRadius, mMaxRadius);
    
    // Print number of Circles found
    if ( circles.size() > 0 ) {
        console() << "Size : " << circles.size() << endl;
        console() << endl;
    }
    
    // Set Current Proccessed Surface
    setProcessedSurface( fromOcv( inputMat ), pCamera );
    
    // Normalise Circles into vector< vector<float> >
    vector< vector<float> > tDots( circles.size(), vector<float>(3) );
    for ( int tPoint = 0; tPoint < circles.size(); tPoint++ ) {
        cv::Vec3i c = circles[tPoint];
        tDots[tPoint][0] = c[0];
        tDots[tPoint][1] = c[1];
        tDots[tPoint][2] = c[2];
    }
    
    // Return the Circles Found
    return tDots;
}

/*
 *  Given two sets of 'mNumTrack' Dots, pairs them by returning them within the same index. Also Orders them clockwise.
 *  Input  : Two sets of 2D Distorted Surface positions, Unpaired.
 *              - Vector< [ [X(2D)0, Y(2D)0, 0], [X(2D)1, Y(2D)1, 0], [0, 0, 0] ] >
 *  Output : Two sets of 2D Distorted Surface positions, Paired.
 *              - Vector< [ [X(2D)0, Y(2D)0, 0], [X(2D)1, Y(2D)1, 0], [0, 0, 0] ] >
 */
void TrackerFinder::pairDots( vector<Image>& pImages ) {
    
    // Store Paired Dots
    vector< vector< vector<float> > > mDotsPaired = mDotsTrio;
    
    // Pair Dots:
    //  - Find Origin of N Dots for both Surfaces
    //  - Calculate Angle to each Dot from corresponding Origin
    //  - Pair by closest matching angles
    
    // Origins of Dots for each Surface
    find2DOrigins( pImages );
    
    // Angles to Dots
    vector< vector<float> > mAngles( mDotsTrio.size(), vector<float>(2) );
    for ( int tCam = 0; tCam < 2; tCam++ ) {
        for ( int tDot = 0; tDot < mDotsTrio.size(); tDot++ ) {
            mAngles[tDot][tCam] = findAngle( mOrigins[tCam], mDotsTrio[tDot][tCam] );
        }
    }
    
    // Match Angles - Output : Vector< [ Angle of First, Index of First, Index of Pair ] >
    vector< vector<float> > mAnglePairs( mDotsTrio.size(), vector<float>(3) );
    //   For each Angle in Cam0, find closest Angle in Cam1
    for ( int tDot = 0; tDot < mDotsTrio.size(); tDot++ ) {
        vector<float> tAnglePair = { mAngles[tDot][0], (float)tDot, 0.0 };
        // Set to first
        float tClosest = abs(mAngles[0][1]);
        int tClosestIndex = 0;
        // Loop through rest
        for ( int tDot2 = 1; tDot2 < mDotsTrio.size(); tDot2++ ) {
            if ( abs(mAngles[tDot2][1] - tAnglePair[0]) < abs(tClosest - tAnglePair[0]) ) {
                tClosest = mAngles[tDot2][1];
                tClosestIndex = tDot2;
            }
        }
        // Set Closest Pair
        tAnglePair[2] = tClosestIndex;
        // Set Pair
        mAnglePairs[tDot] = tAnglePair;
    }
    
    // Check Dots paired up Properly
    vector< int > tCheckSum( mDotsTrio.size(), 0 );
    for ( int tDot = 0; tDot < mDotsTrio.size(); tDot++ ) {
        tCheckSum[mAnglePairs[tDot][2]] = 1;
    }
    int tTotal = 0;
    for ( int tDot = 0; tDot < mDotsTrio.size(); tDot++ ) {
        tTotal += tCheckSum[tDot];
    }
    if ( tTotal != tCheckSum.size() ) {
        if ( VERBOSE_ERROR ) console() << "*** Dots did not Pair Up properly!!! ***" << endl;
        mLastFrameSuccess = false;
    }
    
    // Reorder based on Angle. Clockwise
    sort(mAnglePairs.begin(), mAnglePairs.end(),
         [](const vector<float>& lhs, const vector<float>& rhs) {
             return lhs[0] < rhs[0]; } );
    
    // Turn into something that can be returned. Use Angle/Index's to reorder Cam1 2D positions.
    for ( int tDot = 0; tDot < mDotsTrio.size(); tDot++ ) {
        // Cam 0
        mDotsPaired[tDot][0] = mDotsTrio[ mAnglePairs[tDot][1] ][0];
        // Cam 1
        mDotsPaired[tDot][1] = mDotsTrio[ mAnglePairs[tDot][2] ][1];
    }
    
    // Find Original Predicted Dot
    for ( int i=0 ; i <= mNumTrack; i++ ) {
        if ( mAnglePairs[i][1] == mNumTrack ) {
            mPredictIndex = i;
            break;
        }
    }
    
    // Return Paired/Ordered 2D positons
    mDotsTrio = mDotsPaired;
}

/*
 *  Given an origin and Point, finds the angle to it in degrees.
 *  Input  : Origin and Point
 *  Output : Angle from Origin to Point
 */
float TrackerFinder::findAngle( vector<float> pOrig, vector<float> pPoint ) {
    
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

/*
 *  Give number of trackers, finds the origins.
 */
void TrackerFinder::find2DOrigins( vector<Image>& pImages ) {
    
    if ( mNumTrack == 3 ) {
        
        for ( int tCam = 0; tCam < 2; tCam++ ) {
            //      Find longest edge.
            // Pairs = 0-1, 0-2, 1-2
            vector<float> tDistances ( 3, 0.0 );
            
            // Use Rectified positions
            vector< Leap::Vector > tPositions( 3, Leap::Vector( 0, 0, 0 ) );
            tPositions[0] = pImages[tCam].rectify( Leap::Vector( mDotsTrio[0][tCam][0], mDotsTrio[0][tCam][1], 0 ) );
            tPositions[1] = pImages[tCam].rectify( Leap::Vector( mDotsTrio[1][tCam][0], mDotsTrio[1][tCam][1], 0 ) );
            tPositions[2] = pImages[tCam].rectify( Leap::Vector( mDotsTrio[2][tCam][0], mDotsTrio[2][tCam][1], 0 ) );
            
            // Find 3 2D distances
            tDistances[0] = sqrt( pow( ( tPositions[0][0] - tPositions[1][0] ), 2.0) + pow( ( tPositions[0][1] - tPositions[1][1] ), 2.0) );
            tDistances[1] = sqrt( pow( ( tPositions[0][0] - tPositions[2][0] ), 2.0) + pow( ( tPositions[0][1] - tPositions[2][1] ), 2.0) );
            tDistances[2] = sqrt( pow( ( tPositions[1][0] - tPositions[2][0] ), 2.0) + pow( ( tPositions[1][1] - tPositions[2][1] ), 2.0) );
            
            // Find Longest Distance
            float tDist = tDistances[0];
            vector<int> tPair { 0, 1 };
            int tMissing = 2;
            if ( tDistances[1] > tDist ) {
                tDist = tDistances[1];
                tPair = { 0, 2 };
                tMissing = 1;
            }
            if ( tDistances[2] > tDist ) {
                tDist = tDistances[2];
                tPair = { 1, 2 };
                tMissing = 0;
            }
            
            // Find half way point of this edge
            vector<float> tOrg = { mDotsTrio[tMissing][tCam][0], mDotsTrio[tMissing][tCam][1] };
            vector<float> tMid ( 2, 0.0 );
            tMid[0] = ( mDotsTrio[ tPair[0] ][tCam][ 0 ] + mDotsTrio[ tPair[1] ][tCam][ 0 ] ) / 2.0;
            tMid[1] = ( mDotsTrio[ tPair[0] ][tCam][ 1 ] + mDotsTrio[ tPair[1] ][tCam][ 1 ] ) / 2.0;
            
            mOrigins[tCam][0] = tMid[0];
            mOrigins[tCam][1] = tMid[1];
            
        }
        
    } else if ( mNumTrack == 4 ) {

        for ( int tCam = 0; tCam < 2; tCam++ ) {
            float tOrigin[2] = { 0.0, 0.0 };
            for ( int tDot = 0; tDot < mDotsTrio.size(); tDot++ ) {
                tOrigin[0] += mDotsTrio[tDot][tCam][0];
                tOrigin[1] += mDotsTrio[tDot][tCam][1];
            }
            mOrigins[tCam][0] = tOrigin[0] / mDotsTrio.size();
            mOrigins[tCam][1] = tOrigin[1] / mDotsTrio.size();
        }
        
    } else {
            console() << "Failed to find origins. Don't know how to find for " << mNumTrack << " dots." << endl;
    }

}

/*
 *  Find 3D Real position of Dots, given their Distorted Surfaces and 2D Distorted positions.
 *  Input  : 2D Distorted Surfaces for Camera 0 & 1, and 2D Distorted positions - Vector< [X(2D), Y(2D)] >
 *  Output : 2D Distorted Surface positions, and 3D Real Space positions 
 *              - Vector< [ [X(2D)0, Y(2D)0], [X(2D)1, Y(2D)1], [X(3D), Y(3D), Z(3D)] ] >
 */
void TrackerFinder::find3DPositions( vector<Image>& pImages ) {
    
    //float h_slope = -(tip.x + camera_offset * (2 * i - 1))/tip.y;
    //float v_slope = tip.z/tip.y;
    
    // Store Slopes for each Dot - [ Dots [ Camera [ H, V, 0 ]]]
    vector< Leap::Vector > tSlopes( 2, Leap::Vector(0,0,0) );
    
    // Find and Store slopes for Dots & Calculate 3D positions
    for ( int tDot = 0; tDot < mDotsTrio.size(); tDot++ ) {
        // Calculate Slopes
        tSlopes[0] = pImages[0].rectify( Leap::Vector( mDotsTrio[tDot][0][0], mDotsTrio[tDot][0][1], 0 ) );
        tSlopes[1] = pImages[1].rectify( Leap::Vector( mDotsTrio[tDot][1][0], mDotsTrio[tDot][1][1], 0 ) );
    
        // Calculate X, Y, Z positions

        // Convert to more meaningful variables
        float mH0 = tSlopes[0].x;
        float mH1 = tSlopes[1].x;
        float mV0 = tSlopes[0].y;
        float mV1 = tSlopes[1].y;
        
        // V0 and V1 should be identical... use avg
        float mV = (float)(mV0 + mV1) / 2.0;
        //float mVdiff = (float)(mV0 - mV1);
        
        // Camera Offset
        float mC = 20.0;
        
        // Calculate X, Y, Z !!!
        float mX = mC * ( ( mH1 + mH0 ) / ( mH1 - mH0 ) );
        float mY = -1.0 * ( ( mX + mC ) / ( mH1 ) );
        float mZ = mV * mY;
        
        // Store 3D positions
        mDotsTrio[tDot][2][0] = mX;
        mDotsTrio[tDot][2][1] = mY;
        mDotsTrio[tDot][2][2] = mZ;
    }

}

/*
 *  Using Positions of All Dots in 2D and 3D, finds the Origins and Directions of both 2D.
 *  Also finds predicted dot when tracing 3 dots.
 */
void TrackerFinder::postProcessing2D( vector<Image>& pImages ) {

    vector< vector<float> > tPredictedDot ( 3, vector<float>( 3, 0.0 ) );
    
    if ( mNumTrack == 3 ) {
        
        mDotsTrio.push_back( vector< vector<float> > ( 3, vector<float> ( 3, 0.0 ) ) );
        
        // 2D
        for ( int tCam = 0; tCam < 2; tCam++ ) {
            // *** Origins ***
            //      Find longest edge.
            // Pairs = 0-1, 0-2, 1-2
            vector<float> tDistances ( 3, 0.0 );
            
            // Use Rectified positions
            vector< Leap::Vector > tPositions( 3, Leap::Vector( 0, 0, 0 ) );
            tPositions[0] = pImages[tCam].rectify( Leap::Vector( mDotsTrio[0][tCam][0], mDotsTrio[0][tCam][1], 0 ) );
            tPositions[1] = pImages[tCam].rectify( Leap::Vector( mDotsTrio[1][tCam][0], mDotsTrio[1][tCam][1], 0 ) );
            tPositions[2] = pImages[tCam].rectify( Leap::Vector( mDotsTrio[2][tCam][0], mDotsTrio[2][tCam][1], 0 ) );
            
            // Find 3 2D distances
            tDistances[0] = sqrt( pow( ( tPositions[0][0] - tPositions[1][0] ), 2.0) + pow( ( tPositions[0][1] - tPositions[1][1] ), 2.0) );
            tDistances[1] = sqrt( pow( ( tPositions[0][0] - tPositions[2][0] ), 2.0) + pow( ( tPositions[0][1] - tPositions[2][1] ), 2.0) );
            tDistances[2] = sqrt( pow( ( tPositions[1][0] - tPositions[2][0] ), 2.0) + pow( ( tPositions[1][1] - tPositions[2][1] ), 2.0) );
            
            
            // Find Longest Distance
            float tDist = tDistances[0];
            vector<int> tPair { 0, 1 };
            int tMissing = 2;
            if ( tDistances[1] > tDist ) {
                tDist = tDistances[1];
                tPair = { 0, 2 };
                tMissing = 1;
            }
            if ( tDistances[2] > tDist ) {
                tDist = tDistances[2];
                tPair = { 1, 2 };
                tMissing = 0;
            }
            
            // Find half way point of this edge
            vector<float> tOrg = { mDotsTrio[tMissing][tCam][0], mDotsTrio[tMissing][tCam][1] };
            vector<float> tMid ( 2, 0.0 );
            tMid[0] = ( mDotsTrio[ tPair[0] ][tCam][ 0 ] + mDotsTrio[ tPair[1] ][tCam][ 0 ] ) / 2.0;
            tMid[1] = ( mDotsTrio[ tPair[0] ][tCam][ 1 ] + mDotsTrio[ tPair[1] ][tCam][ 1 ] ) / 2.0;
            
            mOrigins[tCam][0] = tMid[0];
            mOrigins[tCam][1] = tMid[1];
            
            // *** Predicted Dots ***
            tPredictedDot[ tCam ][0] = tMid[0] + ( tMid[0] - tOrg[0] );
            tPredictedDot[ tCam ][1] = tMid[1] + ( tMid[1] - tOrg[1] );
            
            // Add Predicted Dot to main DotsTrio
            mDotsTrio[ mNumTrack ][ tCam ] = { tPredictedDot[ tCam ][0], tPredictedDot[ tCam ][1], 0.0 };
            
            // *** Directions ***
            
            // Need to find angle half way between predicted and first anticlockwise
            float tPredictAngle = findAngle( mOrigins[tCam], tPredictedDot[tCam] );
            // 2 Possible Angles
            //vector<float> tPairAngles = { findAngle( mOrigins[tCam], vector<float> { mDotsTrio[ tPair[0] ][tCam][ 0 ], mDotsTrio[ tPair[0] ][tCam][ 1 ] } ),
            //                              findAngle( mOrigins[tCam], vector<float> { mDotsTrio[ tPair[1] ][tCam][ 0 ], mDotsTrio[ tPair[1] ][tCam][ 1 ] } ) };
            
            /*
            // Find which angle comes first
            float tAngle = tPredictAngle;
            int tMatch;
            while ( true ) {
                if ( abs(tAngle - tPairAngles[0]) < 2.1 ) {
                    tMatch = 0;
                    break;
                }
                if ( abs(tAngle - tPairAngles[1]) < 2.1 ) {
                    tMatch = 1;
                    break;
                }
                tAngle -= 2.0;
                if ( tAngle < 0.0 )
                    tAngle = 360.0;
            }
             */
            
            vector<float> tQuadAngles = { findAngle( mOrigins[tCam], vector<float> { mDotsTrio[ 0 ][tCam][ 0 ], mDotsTrio[ 0 ][tCam][ 1 ] } ),
                                          findAngle( mOrigins[tCam], vector<float> { mDotsTrio[ 1 ][tCam][ 0 ], mDotsTrio[ 1 ][tCam][ 1 ] } ),
                                          findAngle( mOrigins[tCam], vector<float> { mDotsTrio[ 2 ][tCam][ 0 ], mDotsTrio[ 2 ][tCam][ 1 ] } ),
                                          findAngle( mOrigins[tCam], vector<float> { mDotsTrio[ 3 ][tCam][ 0 ], mDotsTrio[ 3 ][tCam][ 1 ] } ) };
            
            // Find which angle comes first
            float tAngle = tPredictAngle;
            int tMatch = 3;
            int tPrevMatch;
            int tCount = BLANK_DOT+0; //+2
            while ( true ) {
                for ( int i=0; i < 4; i++ ) {
                    if ( abs( tAngle - tQuadAngles[i] ) < 2.1 ) {
                        tPrevMatch = tMatch;
                        tMatch = i;
                        tCount --;
                        tAngle += 8.0;
                        break;
                    }
                }
                // Found it so break found
                if ( tCount == 0 ) break;
                // Change Angle, and wrap around is necessary
                tAngle += 2.0;
                if ( tAngle > 360.0 )
                    tAngle = 0.0;
                
            }
            
            vector<float> tMid2 ( 2, 0.0 );
            //tMid2[0] = ( mDotsTrio[ tPair[tMatch] ][tCam][ 0 ] + tPredictedDot[ tCam ][0] ) / 2.0;
            //tMid2[1] = ( mDotsTrio[ tPair[tMatch] ][tCam][ 1 ] + tPredictedDot[ tCam ][1] ) / 2.0;
            tMid2[0] = ( mDotsTrio[ tMatch ][tCam][ 0 ] + mDotsTrio[ tPrevMatch ][tCam][ 0 ] ) / 2.0;
            tMid2[1] = ( mDotsTrio[ tMatch ][tCam][ 1 ] + mDotsTrio[ tPrevMatch ][tCam][ 1 ] ) / 2.0;
            
            mDirections[ tCam ] = findAngle( mOrigins[tCam], tMid2 );
            
            /*
             // Amend based on blank dot
             switch ( BLANK_DOT ) {
             case 1: // Top Left
             mDirections[ tCam ] = myMod( mDirections[ tCam ] + 45.0, 360 );
             break;
             case 2: // Top Right
             mDirections[ tCam ] = myMod( mDirections[ tCam ] - 45.0, 360 );
             break;
             case 3: // Bottom Right
             mDirections[ tCam ] = myMod( mDirections[ tCam ] - 135.0, 360 );
             break;
             case 4: // Bottom Left
             mDirections[ tCam ] = myMod( mDirections[ tCam ] + 135.0, 360 );
             break;
             }*/
            
        }
        
    } else if ( mNumTrack == 4 ) {
        
        // *** Origins ***
        // 2D
        
        for ( int tCam = 0; tCam < 2; tCam++ ) {
            float tOrigin[2] = { 0.0, 0.0 };
            for ( int tDot = 0; tDot < mDotsTrio.size(); tDot++ ) {
                tOrigin[0] += mDotsTrio[tDot][tCam][0];
                tOrigin[1] += mDotsTrio[tDot][tCam][1];
            }
            mOrigins[tCam][0] = tOrigin[0] / mDotsTrio.size();
            mOrigins[tCam][1] = tOrigin[1] / mDotsTrio.size();
        }
        
    } else {
        console() << "Failed to find origins. Don't know how to find for " << mNumTrack << " dots." << endl;
    }
    
}

/*
 *  Using Positions of All Dots in 2D and 3D, finds the Origins and Directions of 3D.
 *  Also finds predicted dot when tracing 3 dots.
 */
void TrackerFinder::postProcessing3D( vector<Image>& pImages ) {
    
    // *** Origins ***
    // 3D
    
    if ( mNumTrack == 3 || mNumTrack == 4 ) {
    
        int tCam = 2;
        if ( mDotsTrio.size() > 0 ) {
            for ( int mDim = 0; mDim < 3; mDim++ ) {
                float tFloat = 0.0;
                for ( int tDot = 0; tDot < mDotsTrio.size(); tDot++ ) {
                    tFloat += mDotsTrio[ tDot ][ 2 ][ mDim ];
                }
                tFloat /= mDotsTrio.size();
                mOrigins[tCam][mDim] = tFloat;
            }
        }
        
        if ( mNumTrack == 3 ) {
            
            // *** Directions ***
            
            // Get direction from point not on longest edge and center of longest edge
            mDirections[2] = findAngle( vector<float> { mOrigins[ tCam ][0], mOrigins[ tCam] [2] },
                                        vector<float> { mDotsTrio[ mPredictIndex ][tCam][ 0 ], mDotsTrio[ mPredictIndex ][tCam][ 2 ] } );
            
            // Amend based on blank dot
            switch ( BLANK_DOT ) {
                case 1: // Top Right
                    mDirections[ tCam ] = myMod( mDirections[ tCam ] + 45.0, 360 );
                    break;
                case 2: // Bottom Right
                    mDirections[ tCam ] = myMod( mDirections[ tCam ] + 135.0, 360 );
                    break;
                case 3: // Bottom Left
                    mDirections[ tCam ] = myMod( mDirections[ tCam ] - 135.0, 360 );
                    break;
                case 4: // Top Left
                    mDirections[ tCam ] = myMod( mDirections[ tCam ] - 45.0, 360 );
                    break;
            }
            
        }
        
    } else {
        console() << "Failed to find origins. Don't know how to find for " << mNumTrack << " dots." << endl;
    }
    
}

/*
 *  Test OpenCV procedures
 *  Input  : Surface, and the Camera to which is originated
 */
void TrackerFinder::processOpenCV( Surface pSurface, int pCamera ) {
    
    // Convert Surface to OpenCV Mat
    cv::Mat inputMat( toOcv( Channel( pSurface ) ) );
    
    // Sharpen Image
    //cv::Mat inputMatSharp;
    //cv::GaussianBlur( inputMat, inputMatSharp, cv::Size(9, 9), 2, 2 );
    //cv::addWeighted(inputMat, 1.5, inputMatSharp, -0.5, 0, inputMat);
    
    cv::threshold(inputMat, inputMat, 60.0, 255, 0);
    
    // Set Current Proccessed Surface
    setProcessedSurface( fromOcv( inputMat ), pCamera );
}

/*
 *
 */
void TrackerFinder::setProcessedSurface( Surface pSurface, int pCamera ) {
    currentProcessedSurface[pCamera] = pSurface;
    processedSurfaceSet[pCamera] = true;
}

/*
 *
 */
vector< Surface > TrackerFinder::getProcessedSurfaces() {
    if ( processedSurfaceSet[0] && processedSurfaceSet[1] ) {
        return currentProcessedSurface;
    }
    console() << "Current Processed Surfaces Not Set!" << endl;
    return vector< Surface >();
}

/*
 *
 */
Surface TrackerFinder::getProcessedSurface( int pCamera ) {
    if ( processedSurfaceSet[pCamera] ) {
        return currentProcessedSurface[pCamera];
    }
    console() << "Current Processed Surface " << pCamera <<  " Not Set!" << endl;
    return Surface();
}

/*
 *
 */
bool TrackerFinder::lastFrameSuccess() {
    return mLastFrameSuccess;
}

/*
 *
 */
void TrackerFinder::setTrackNum( int pNum ) {
    mNumTrack = pNum;
}

/*
 *
 */
float TrackerFinder::myMod( float pVal, int pMod ) {
    float tVal = pVal;
    
    while ( tVal < 0 || tVal > pMod ) {
        if (tVal < 0) tVal += pMod;
        if (tVal > pMod) tVal -= pMod;
    }
    
    return tVal;
}

/*
 *
 */
vector< vector<float> > TrackerFinder::getOrigins() {
    return mOrigins;
}

/*
 *
 */
vector<float> TrackerFinder::getDirections() {
    return mDirections;
}