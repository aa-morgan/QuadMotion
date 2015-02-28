/*
 *  Class - Arduino
 *  Author - Alex Morgan
 */

#include "Arduino.h"

/*
 *  Constructor
 */
Arduino::Arduino() {
    
}

/*
 *  Setup
 */
void Arduino::setup() {
    
	// print the devices available
	const vector<Serial::Device> &devices( Serial::getDevices() );
    console() << "Devices available:" << endl;
	for( vector<Serial::Device>::const_iterator deviceIt = devices.begin(); deviceIt != devices.end(); ++deviceIt ) {
        console() << "\tDevice: " << deviceIt->getName() << endl;
	}
	
    console() << "Attempting to connect to 'tty.usbmodem14..' with " << mBaudRate << " Baud Rate..." << endl;
    while ( !mSetupSucess ) {
        
        // Attempt to Connect to TTY
        try {
            
            Serial::Device dev = Serial::findDeviceByNameContains("tty.usbmodem14");
            mSerial = Serial( dev, mBaudRate );
            console() << "Successfully connected!" << endl;
            mSetupSucess = true;
        }
        catch( ... ) {
            console() << "Failed. Retrying..." << endl;
            usleep( 2000000 );
        }
        
    }
    
    // Wait for Arduino to setup
    usleep( 1500000 );
    
    // Flush Serial Port
	mSerial.flush();
    
    // Tell Arduino that we have connected successfully
    Byte tByte = CONNECT_PACKET;
    mSerial.writeByte( tByte );
    
    // Wait for Quad to setup
    usleep( 5000000 );
    
}

/*
 *  Send
 *      Throttle - Fly Up & Down - [ 0->1 ]
 *      Rudder   - Rotate Clockwise & Anti-Clockwise - [ -1->1 ]
 *      Elevator - Pitch Forward & Backwards - [ -1->1 ]
 *      Aileron  - Roll Left & Right - [ -1->1 ]
 */
void Arduino::send( vector<float> pControl ) {
    
    // Copies to work with
    vector<float> tControlIn = pControl;
    vector<int> tControlOut( pControl.size(), 0 );
    
    // Sanitise Controls
    if ( tControlIn[0] > 1.0 )
        tControlIn[0] = 1.0;
    if ( tControlIn[0] < 0.0 )
        tControlIn[0] = 0.0;
    for ( int i=1; i < tControlIn.size(); i++ ) {
        if ( tControlIn[i] > 1.0 )
            tControlIn[i] = 1.0;
        if ( tControlIn[i] < -1.0 )
            tControlIn[i] = -1.0;
    }
    
    // Map Values from 0-1,-1,1,-1,1,-1,1 to 127 +/- mRange
    //tControlOut[0] = 230.0 + ( tControlIn[0] * 20.0 );
    tControlOut[0] = ( tControlIn[0] * 200 ); // TEST
    
    for ( int i=1; i < tControlIn.size(); i++ ) {
        tControlOut[i] = ( tControlIn[i] * mRange ) + mMid;
        //tControlOut[i] = 127; // TEST
    }
    
    // Send converted results.
    sendRaw( tControlOut );
    
}

/*
 *  Send Raw
 *      Throttle - Fly Up & Down - [ 0->255 ]
 *      Rudder   - Rotate Clockwise & Anti-Clockwise - [ 0->255 ]
 *      Elevator - Pitch Forward & Backwards - [ 0->255 ]
 *      Aileron  - Roll Left & Right - [ 0->255 ]
 */
void Arduino::sendRaw( vector<int> pControl ) {
    
    if ( pControl.size() != 4 ) {
        console() << "Control vector is not 4 bytes!" << endl;
    } else {
        console() << "Writing to Arduino : " << CONTROL_PACKET << " " << pControl[0] << " " << pControl[1] << " " << pControl[2] << " " << pControl[3] << endl;
        Byte mBytes[5] = { (Byte)CONTROL_PACKET, (Byte)pControl[0], (Byte)pControl[1], (Byte)pControl[2], (Byte)pControl[3] };
        mSerial.writeBytes( mBytes, 5 );
    }
    
}
