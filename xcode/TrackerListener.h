#ifndef __TrackDots__TrackerListener__
#define __TrackDots__TrackerListener__

#include "Resources.h"

using namespace Leap;
using namespace std;
using namespace ci;
using namespace ci::app;

/*
 *
 */
class TrackerListener : public Leap::Listener {
public:
    virtual void onInit(const Leap::Controller&);
    virtual void onConnect(const Leap::Controller&);
    virtual void onDisconnect(const Leap::Controller&);
    virtual void onExit(const Leap::Controller&);
    virtual void onFrame(const Leap::Controller&);
    virtual void onFocusGained(const Leap::Controller&);
    virtual void onFocusLost(const Leap::Controller&);
    virtual void onDeviceChange(const Leap::Controller&);
    virtual void onServiceConnect(const Leap::Controller&);
    virtual void onServiceDisconnect(const Leap::Controller&);
    
    vector<Surface> getSurfaces();
    Surface getSurface( int );
    void setSurface( int, Surface) ;
    vector<Image> getImages();
    void setImage( int, Image );
    Frame getFrame();
    void setFrame(Frame);
    
private:
    Frame currentFrame;
    Image currentImages[2];
    Surface currentSurfaces[2];
    
    bool firstFrameSet = false;
    bool firstImageSet[2] = {false};
    bool firstSurfaceSet[2] = {false};
};

#endif /* defined(__TrackDots__TrackerListener__) */
