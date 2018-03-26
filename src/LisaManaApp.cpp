#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class LisaManaApp : public App {
  public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void update() override;
	void draw() override;
};

void LisaManaApp::setup()
{
}

void LisaManaApp::mouseDown( MouseEvent event )
{
}

void LisaManaApp::update()
{
}

void LisaManaApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) ); 
}

CINDER_APP( LisaManaApp, RendererGl )
