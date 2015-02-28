/*
 *  Class - TrackerListener
 *  Author - Alex Morgan
 */

#include "TrackerListener.h"

/*
 *
 */
void TrackerListener::onInit(const Controller& controller) {
    console() << "Initialized" << std::endl;
}

/*
 *
 */
void TrackerListener::onConnect(const Controller& controller) {
    console() << "Connected" << std::endl;
}

/*
 *
 */
void TrackerListener::onDisconnect(const Controller& controller) {
    // Note: not dispatched when running in a debugger.
    console() << "Disconnected" << std::endl;
}

/*
 *
 */
void TrackerListener::onExit(const Controller& controller) {
    console() << "Exited" << std::endl;
}

/*
 *
 */
void TrackerListener::onFrame(const Controller& controller) {
    
    //Uses Cinder OpenGL wrapper
    Frame frame = controller.frame();
    setFrame( frame );
    
    ImageList images = frame.images();
    for(int i = 0; i < 2; i++){
        Image image = images[i];
        
        const unsigned char* image_buffer = image.data();
        
        //Draw the raw image data as a greyscale bitmap
        Surface surface(image.width(), image.height(), image.width() * 4, SurfaceChannelOrder::RGBA);
        int cursor = 0;
        Surface::Iter iter = surface.getIter();
        while( iter.line() ) {
            while( iter.pixel() ) {
                iter.r() = image_buffer[cursor];
                iter.g() = iter.b() = iter.r();
                iter.a() = 255;
                cursor++;
            }
        }
        
        // Set Current Image
        setImage( i, image );
        
        // Set Current Surface
        setSurface(i, surface);
    }
    
}

/*
 *
 */
void TrackerListener::onFocusGained(const Controller& controller) {
    console() << "Focus Gained" << std::endl;
}

/*
 *
 */
void TrackerListener::onFocusLost(const Controller& controller) {
    console() << "Focus Lost" << std::endl;
}

/*
 *
 */
void TrackerListener::onDeviceChange(const Controller& controller) {
    console() << "Device Changed" << std::endl;
    const DeviceList devices = controller.devices();
    
    for (int i = 0; i < devices.count(); ++i) {
        console() << "id: " << devices[i].toString() << std::endl;
        console() << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
    }
}

/*
 *
 */
void TrackerListener::onServiceConnect(const Controller& controller) {
    console() << "Service Connected" << std::endl;
}

/*
 *
 */
void TrackerListener::onServiceDisconnect(const Controller& controller) {
    console() << "Service Disconnected" << std::endl;
}

/*
 *
 */
vector< Surface > TrackerListener::getSurfaces() {
    // Wait until first & second Surface has been set
    while ( firstSurfaceSet[0] == false || firstSurfaceSet[1] == false ) {}
    
    while ( currentSurfaces[0].getWidth() == 0 ){}
    while ( currentSurfaces[1].getWidth() == 0 ){}

    return vector< Surface > { currentSurfaces[0], currentSurfaces[1] };
}

/*
 *
 */
Surface TrackerListener::getSurface(int pCamera) {
    // Wait until first Surface has been set
    while ( firstSurfaceSet[pCamera] == false ) {}

    if (pCamera == 0) {
        // Wait until valid Surface
        while ( currentSurfaces[pCamera].getWidth() == 0 ){}
        return currentSurfaces[pCamera];
    } else if (pCamera == 1) {
        // Wait until valid Surface
        while ( currentSurfaces[pCamera].getWidth() == 0 ){}
        return currentSurfaces[pCamera];
    }
    // Return empty Surface if wrong pCamera
    return Surface();
}

/*
 *
 */
void TrackerListener::setSurface(int pCamera, Surface pSurface) {
    if (pCamera == 0) {
        currentSurfaces[pCamera] = pSurface;
        // Signify that first SurfaceLeft has been set
        firstSurfaceSet[pCamera] = true;
    } else if (pCamera == 1) {
        currentSurfaces[pCamera] = pSurface;
        // Signify that first SurfaceRight has been set
        firstSurfaceSet[pCamera] = true;
    }
}

/*
 *
 */
vector<Image> TrackerListener::getImages() {
    // Wait until First & Second Image has been set
    while ( firstImageSet[0] == false || firstImageSet[1] == false ) {}
    
    while ( currentImages[0].width() == 0 ){}
    while ( currentImages[1].width() == 0 ){}
    
    return vector<Image> { currentImages[0], currentImages[1] };
}

/*
 *
 */
void TrackerListener::setImage( int pCamera, Image pImage ) {
    if (pCamera == 0) {
        currentImages[pCamera] = pImage;
        // Signify that first ImageLeft has been set
        firstImageSet[pCamera] = true;
    } else if (pCamera == 1) {
        currentImages[pCamera] = pImage;
        // Signify that first ImageRight has been set
        firstImageSet[pCamera] = true;
    }
}

/*
 *
 */
Frame TrackerListener::getFrame() {
    // Wait untill First Fram has been set
    while ( firstFrameSet == false ) {}
    
    return currentFrame;
}

/*
 *
 */
void TrackerListener::setFrame(Frame pFrame) {
    currentFrame = pFrame;
    // Signify that first frame has been set
    firstFrameSet = true;
}


