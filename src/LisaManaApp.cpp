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

	// points for 1st person pentagon
	vec2 pointA1;
	vec2 pointA2;
	vec2 pointA3;
	vec2 pointA4;
	vec2 pointA5;

	// points for 2nd person pentagon
	vec2 pointB1;
	vec2 pointB2;
	vec2 pointB3;
	vec2 pointB4;
	vec2 pointB5;

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
		int counter = 0;
		for (const Kinect2::Body &body : mBodyFrame.getBodies()) {
			if (body.isTracked()) {
				gl::color(ColorAf::white());

				auto map = body.getJointMap();
				vec3 point1 = (map.at(JointType_Head).getPosition() + map.at(JointType_SpineShoulder).getPosition()) / vec3(2);
				vec3 point2 = (map.at(JointType_ShoulderRight).getPosition() + map.at(JointType_ElbowRight).getPosition() + map.at(JointType_WristRight).getPosition() + map.at(JointType_HandRight).getPosition()) / vec3(4);
				vec3 point3 = (map.at(JointType_ShoulderLeft).getPosition() + map.at(JointType_ElbowLeft).getPosition() + map.at(JointType_WristLeft).getPosition() + map.at(JointType_HandLeft).getPosition()) / vec3(4);
				vec3 point4 = (map.at(JointType_HipRight).getPosition() + map.at(JointType_KneeRight).getPosition() + map.at(JointType_AnkleRight).getPosition() + map.at(JointType_FootRight).getPosition()) / vec3(4);
				vec3 point5 = (map.at(JointType_HipLeft).getPosition() + map.at(JointType_KneeLeft).getPosition() + map.at(JointType_AnkleLeft).getPosition() + map.at(JointType_FootLeft).getPosition()) / vec3(4);

				for (const auto& joint : map) {
				//	console() << joint.first << endl;
					if (joint.second.getTrackingState() == TrackingState::TrackingState_Tracked) {
						vec2 pos(mDevice->mapCameraToDepth(joint.second.getPosition()));
						if (mDrawSkeleton) {
							gl::drawSolidCircle(pos, 5.0f, 32);
							vec2 parent(mDevice->mapCameraToDepth(map.at(joint.second.getParentJoint()).getPosition()));
							gl::drawLine(pos, parent);
						}
					}
				}
				if (mDrawPentagon) {
					// calculate positions which we will use to calculate the five vertices of the pentagons
					vec2 headPos(mDevice->mapCameraToDepth(map.at(JointType_Head).getPosition()));
					vec2 shoulderPos(mDevice->mapCameraToDepth(map.at(JointType_SpineShoulder).getPosition()));

					vec2 shoulderRPos(mDevice->mapCameraToDepth(map.at(JointType_ShoulderRight).getPosition()));
					vec2 elbowRPos(mDevice->mapCameraToDepth(map.at(JointType_ElbowRight).getPosition()));
					vec2 wristRPos(mDevice->mapCameraToDepth(map.at(JointType_WristRight).getPosition()));
					vec2 handRPos(mDevice->mapCameraToDepth(map.at(JointType_HandRight).getPosition()));

					vec2 shoulderLPos(mDevice->mapCameraToDepth(map.at(JointType_ShoulderLeft).getPosition()));
					vec2 elbowLPos(mDevice->mapCameraToDepth(map.at(JointType_ElbowLeft).getPosition()));
					vec2 wristLPos(mDevice->mapCameraToDepth(map.at(JointType_WristLeft).getPosition()));
					vec2 handLPos(mDevice->mapCameraToDepth(map.at(JointType_HandLeft).getPosition()));

					vec2 hipRPos(mDevice->mapCameraToDepth(map.at(JointType_HipRight).getPosition()));
					vec2 kneeRPos(mDevice->mapCameraToDepth(map.at(JointType_KneeRight).getPosition()));
					vec2 ankleRPos(mDevice->mapCameraToDepth(map.at(JointType_AnkleRight).getPosition()));
					vec2 footRPos(mDevice->mapCameraToDepth(map.at(JointType_FootRight).getPosition()));

					vec2 hipLPos(mDevice->mapCameraToDepth(map.at(JointType_HipLeft).getPosition()));
					vec2 kneeLPos(mDevice->mapCameraToDepth(map.at(JointType_KneeLeft).getPosition()));
					vec2 ankleLPos(mDevice->mapCameraToDepth(map.at(JointType_AnkleLeft).getPosition()));
					vec2 footLPos(mDevice->mapCameraToDepth(map.at(JointType_FootLeft).getPosition()));

					gl::lineWidth(5.0f);

					// draw pentagon for 1st person
					if (counter == 0) {
						gl::color(1, 0, 0);
						
						pointA1 = (headPos + shoulderPos) / vec2(2);
						gl::drawSolidCircle(pointA1, 5.0f, 32);

						
						pointA2 = (shoulderRPos + elbowRPos + wristRPos + handRPos) / vec2(4);
						gl::drawSolidCircle(pointA2, 5.0f, 32);

						
						pointA3 = (shoulderLPos + elbowLPos + wristLPos + handLPos) / vec2(4);
						gl::drawSolidCircle(pointA3, 5.0f, 32);

						pointA4 = (hipRPos + kneeRPos + ankleRPos + footRPos) / vec2(4);
						gl::drawSolidCircle(pointA4, 5.0f, 32);
						
						pointA5 = (hipLPos + kneeLPos + ankleLPos + footLPos) / vec2(4);
						gl::drawSolidCircle(pointA5, 5.0f, 32);

						gl::drawLine(pointA1, pointA2);
						gl::drawLine(pointA2, pointA4);
						gl::drawLine(pointA4, pointA5);
						gl::drawLine(pointA5, pointA3);
						gl::drawLine(pointA3, pointA1);
					}
					// draw pentagon for 2nd person
					else if (counter == 1) {
						gl::color(0, 1, 0);

						pointB1 = (headPos + shoulderPos) / vec2(2);
						gl::drawSolidCircle(pointB1, 5.0f, 32);


						pointB2 = (shoulderRPos + elbowRPos + wristRPos + handRPos) / vec2(4);
						gl::drawSolidCircle(pointB2, 5.0f, 32);


						pointB3 = (shoulderLPos + elbowLPos + wristLPos + handLPos) / vec2(4);
						gl::drawSolidCircle(pointB3, 5.0f, 32);

						pointB4 = (hipRPos + kneeRPos + ankleRPos + footRPos) / vec2(4);
						gl::drawSolidCircle(pointB4, 5.0f, 32);

						pointB5 = (hipLPos + kneeLPos + ankleLPos + footLPos) / vec2(4);
						gl::drawSolidCircle(pointB5, 5.0f, 32);

						gl::drawLine(pointB1, pointB2);
						gl::drawLine(pointB2, pointB4);
						gl::drawLine(pointB4, pointB5);
						gl::drawLine(pointB5, pointB3);
						gl::drawLine(pointB3, pointB1);

						// calculate distances
						int dist1 = sqrt(math<float>::pow(pointA1.x - pointB1.x, 2) + math<float>::pow(pointA1.y - pointB1.y, 2));
						int dist2 = sqrt(math<float>::pow(pointA2.x - pointB2.x, 2) + math<float>::pow(pointA2.y - pointB2.y, 2));
						int dist3 = sqrt(math<float>::pow(pointA3.x - pointB3.x, 2) + math<float>::pow(pointA3.y - pointB3.y, 2));
						int dist4 = sqrt(math<float>::pow(pointA4.x - pointB4.x, 2) + math<float>::pow(pointA4.y - pointB4.y, 2));
						int dist5 = sqrt(math<float>::pow(pointA5.x - pointB5.x, 2) + math<float>::pow(pointA5.y - pointB5.y, 2));
					}
				}
				drawHand(body.getHandLeft(), mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HandLeft).getPosition()));
				drawHand(body.getHandRight(), mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HandRight).getPosition()));

				counter++;
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
