#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Kinect2.h"
#include "cinder/params/Params.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class LisaManaApp : public App {
  public:
	LisaManaApp();
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void update() override;
	void draw() override;
	void shutdown();

private:
	Kinect2::DeviceRef mDevice;
	Kinect2::BodyFrame mBodyFrame;
	ci::Channel8uRef mChannelBodyIndex;
	ci::Channel16uRef mChannelDepth;

	params::InterfaceGlRef mParams;
	bool mDrawBackground = true;
	bool mDrawSkeleton = true;
	bool mDrawPentagon = true;
};

LisaManaApp::LisaManaApp() {
	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectBodyEventHandler([&](const Kinect2::BodyFrame frame) {
		mBodyFrame = frame;
	});
	mDevice->connectBodyIndexEventHandler([&](const Kinect2::BodyIndexFrame frame) {
		mChannelBodyIndex = frame.getChannel();
	});
	mDevice->connectDepthEventHandler([&](const Kinect2::DepthFrame frame) {
		mChannelDepth = frame.getChannel();
	});
}

void LisaManaApp::setup()
{
	// create a parameter interface and name it
	mParams = params::InterfaceGl::create(getWindow(), "App parameters", toPixels(ivec2(200, 400)));

	// setup parameters
	mParams->addParam("Draw background", &mDrawBackground);
	mParams->addParam("Draw Skeleton", &mDrawSkeleton);
	mParams->addParam("Draw Pentagon", &mDrawPentagon);
}

void LisaManaApp::mouseDown( MouseEvent event )
{
}

void LisaManaApp::update()
{
}

void LisaManaApp::draw()
{
	const gl::ScopedViewport scopedViewport(ivec2(0), getWindowSize());
	const gl::ScopedMatrices scopedMatrices;
	const gl::ScopedBlendAlpha scopedBlendAlpha;
	gl::setMatricesWindow(getWindowSize());
	gl::clear();
	gl::color(ColorAf::white());
	gl::disableDepthRead();
	gl::disableDepthWrite();

	if (mChannelDepth && mDrawBackground) {
		gl::enable(GL_TEXTURE_2D);
		const gl::TextureRef tex = gl::Texture::create(*Kinect2::channel16To8(mChannelDepth));
		gl::draw(tex, tex->getBounds(), Rectf(getWindowBounds()));
	}

	if (mChannelBodyIndex) {
		gl::enable(GL_TEXTURE_2D);

		auto drawHand = [&](const Kinect2::Body::Hand& hand, const ivec2& pos) -> void
		{
			switch (hand.getState()) {
			case HandState_Closed:
				gl::color(ColorAf(1.0f, 0.0f, 0.0f, 0.5f));
				break;
			case HandState_Lasso:
				gl::color(ColorAf(0.0f, 0.0f, 1.0f, 0.5f));
				break;
			case HandState_Open:
				gl::color(ColorAf(0.0f, 1.0f, 0.0f, 0.5f));
				break;
			default:
				gl::color(ColorAf(0.0f, 0.0f, 0.0f, 0.0f));
				break;
			}
			gl::drawSolidCircle(pos, 30.0f, 32);
		};

		gl::pushMatrices();
		gl::scale(vec2(getWindowSize()) / vec2(mChannelBodyIndex->getSize()));
		gl::disable(GL_TEXTURE_2D);
		for (const Kinect2::Body &body : mBodyFrame.getBodies()) {
			if (body.isTracked()) {
				gl::color(ColorAf::white());
				for (const auto& joint : body.getJointMap()) {
				//	console() << joint.first << endl;
					if (joint.second.getTrackingState() == TrackingState::TrackingState_Tracked) {
						vec2 pos(mDevice->mapCameraToDepth(joint.second.getPosition()));
						if (mDrawSkeleton) {
							gl::drawSolidCircle(pos, 5.0f, 32);
							vec2 parent(mDevice->mapCameraToDepth(body.getJointMap().at(joint.second.getParentJoint()).getPosition()));
							gl::drawLine(pos, parent);
						}
					}
				}
				if (mDrawBackground) {
					gl::color(1, 0, 0);
					vec2 headPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_Head).getPosition()));
					vec2 shoulderPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_SpineShoulder).getPosition()));
					vec2 point1 = (headPos + shoulderPos) / vec2(2);
					gl::drawSolidCircle(point1, 5.0f, 32);

					vec2 shoulderRPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_ShoulderRight).getPosition()));
					vec2 elbowRPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_ElbowRight).getPosition()));
					vec2 wristRPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_WristRight).getPosition()));
					vec2 handRPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HandRight).getPosition()));
					vec2 point2 = (shoulderRPos + elbowRPos + wristRPos + handRPos) / vec2(4);
					gl::drawSolidCircle(point2, 5.0f, 32);

					vec2 shoulderLPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_ShoulderLeft).getPosition()));
					vec2 elbowLPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_ElbowLeft).getPosition()));
					vec2 wristLPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_WristLeft).getPosition()));
					vec2 handLPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HandLeft).getPosition()));
					vec2 point3 = (shoulderLPos + elbowLPos + wristLPos + handLPos) / vec2(4);
					gl::drawSolidCircle(point3, 5.0f, 32);

					vec2 hipRPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HipRight).getPosition()));
					vec2 kneeRPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_KneeRight).getPosition()));
					vec2 ankleRPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_AnkleRight).getPosition()));
					vec2 footRPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_FootRight).getPosition()));
					vec2 point4 = (hipRPos + kneeRPos + ankleRPos + footRPos) / vec2(4);
					gl::drawSolidCircle(point4, 5.0f, 32);

					vec2 hipLPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HipLeft).getPosition()));
					vec2 kneeLPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_KneeLeft).getPosition()));
					vec2 ankleLPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_AnkleLeft).getPosition()));
					vec2 footLPos(mDevice->mapCameraToDepth(body.getJointMap().at(JointType_FootLeft).getPosition()));
					vec2 point5 = (hipLPos + kneeLPos + ankleLPos + footLPos) / vec2(4);
					gl::drawSolidCircle(point5, 5.0f, 32);

					gl::lineWidth(5.0f);
					gl::drawLine(point1, point2);
					gl::drawLine(point2, point4);
					gl::drawLine(point4, point5);
					gl::drawLine(point5, point3);
					gl::drawLine(point3, point1);
				}
				drawHand(body.getHandLeft(), mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HandLeft).getPosition()));
				drawHand(body.getHandRight(), mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HandRight).getPosition()));
			}
		}
	}

	// draw parameters interface
	mParams->draw();
}

void LisaManaApp::shutdown() {
	mDevice->stop();
}

CINDER_APP( LisaManaApp, RendererGl )
