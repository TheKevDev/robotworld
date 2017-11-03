#include "Robot.hpp"
#include <sstream>
#include <ctime>
#include <chrono>
#include "Thread.hpp"
#include "MathUtils.hpp"
#include "Logger.hpp"
#include "Goal.hpp"
#include "WayPoint.hpp"
#include "Wall.hpp"
#include "RobotWorld.hpp"
#include "Shape2DUtils.hpp"
#include "CommunicationService.hpp"
#include "Client.hpp"
#include "Message.hpp"
#include "MainFrameWindow.hpp"
#include "MainApplication.hpp"
#include "LaserDistanceSensor.hpp"

namespace Model {

/**
 *
 */
Robot::Robot() :
		name(""), size( DefaultSize), position( DefaultPosition), front(0, 0), speed(
				0.0), acting(false), driving(false), communicating(false) {
	std::shared_ptr<AbstractSensor> laserSensor(new LaserDistanceSensor(this));
	attachSensor(laserSensor);
}

/**
 *
 */
Robot::Robot(const std::string& aName) :
		name(aName), size( DefaultSize), position( DefaultPosition), front(0,
				0), speed(0.0), acting(false), driving(false), communicating(
				false) {
	std::shared_ptr<AbstractSensor> laserSensor(new LaserDistanceSensor(this));
	attachSensor(laserSensor);
}

/**
 *
 */
Robot::Robot(const std::string& aName, const Point& aPosition) :
		name(aName), size( DefaultSize), position(aPosition), front(0, 0), speed(
				0.0), acting(false), driving(false), communicating(false) {
	std::shared_ptr<AbstractSensor> laserSensor(new LaserDistanceSensor(this));
	attachSensor(laserSensor);
}

/**
 *
 */
Robot::~Robot() {
	if (driving) {
		stopDriving();
	}
	if (acting) {
		stopActing();
	}
	if (communicating) {
		stopCommunicating();
	}
}

/**
 *
 */
void Robot::setName(const std::string& aName,
		bool aNotifyObservers /*= true*/) {
	name = aName;
	if (aNotifyObservers == true) {
		notifyObservers();
	}

}

/**
 *
 */
Size Robot::getSize() const {
	return size;
}

/**
 *
 */
void Robot::setSize(const Size& aSize, bool aNotifyObservers /*= true*/) {
	size = aSize;
	if (aNotifyObservers == true) {
		notifyObservers();
	}
}

/**
 *
 */
void Robot::setPosition(const Point& aPosition,
		bool aNotifyObservers /*= true*/) {
	position = aPosition;
	if (aNotifyObservers == true) {
		notifyObservers();
	}
}

/**
 *
 */
BoundedVector Robot::getFront() const {
	return front;
}

/**
 *
 */
void Robot::setFront(const BoundedVector& aVector,
		bool aNotifyObservers /*= true*/) {
	front = aVector;
	if (aNotifyObservers == true) {
		notifyObservers();
	}
}

/**
 *
 */
float Robot::getSpeed() const {
	return speed;
}

/**
 *
 */
void Robot::setSpeed(float aNewSpeed, bool aNotifyObservers /*= true*/) {
	speed = aNewSpeed;
	if (aNotifyObservers == true) {
		notifyObservers();
	}
}

/**
 *
 */
void Robot::startActing() {
	acting = true;
	std::thread newRobotThread([this] {startDriving();});
	robotThread.swap(newRobotThread);
}

/**
 *
 */
void Robot::stopActing() {
	acting = false;
	driving = false;
	robotThread.join();
}

/**
 *
 */
void Robot::startDriving() {
	driving = true;

	goal = RobotWorld::getRobotWorld().getGoal("Goal");
	calculateRoute(goal);

	drive();
}

/**
 *
 */
void Robot::stopDriving() {
	driving = false;
}

/**
 *
 */
void Robot::startCommunicating() {
	if (!communicating) {
		communicating = true;

		std::string localPort = "12345";
		if (Application::MainApplication::isArgGiven("-local_port")) {
			localPort =
					Application::MainApplication::getArg("-local_port").value;
		}

		if (Application::MainApplication::isArgGiven("-robot_type")) {
			if (Application::MainApplication::getArg("-robot_type").value
					== "client") {
				localPort = "12346";
			}
		}

		Application::Logger::log("Mijn port is: " + localPort);

		Messaging::CommunicationService::getCommunicationService().runRequestHandler(
				toPtr<Robot>(), std::stoi(localPort));
	}
}

/**
 *
 */
void Robot::stopCommunicating() {
	if (communicating) {
		communicating = false;

		std::string localPort = "12345";
		if (Application::MainApplication::isArgGiven("-local_port")) {
			localPort =
					Application::MainApplication::getArg("-local_port").value;
		}

		Messaging::Client c1ient("localhost", localPort, toPtr<Robot>());
		Messaging::Message message(1, "stop");
		c1ient.dispatchMessage(message);
	}
}

/**
 *
 */
Region Robot::getRegion() const {
	Point translatedPoints[] = { getFrontRight(), getFrontLeft(), getBackLeft(),
			getBackRight() };
	return Region(4, translatedPoints);
}

/**
 *
 */
bool Robot::intersects(const Region& aRegion) const {
	Region region = getRegion();
	region.Intersect(aRegion);
	return !region.IsEmpty();
}

/**
 *
 */
Point Robot::getFrontLeft() const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalFrontLeft(x, y);
	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point frontLeft(
			(originalFrontLeft.x - position.x) * std::cos(angle)
					- (originalFrontLeft.y - position.y) * std::sin(angle)
					+ position.x,
			(originalFrontLeft.y - position.y) * std::cos(angle)
					+ (originalFrontLeft.x - position.x) * std::sin(angle)
					+ position.y);

	return frontLeft;
}

/**
 *
 */
Point Robot::getFrontRight() const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalFrontRight(x + size.x, y);
	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point frontRight(
			(originalFrontRight.x - position.x) * std::cos(angle)
					- (originalFrontRight.y - position.y) * std::sin(angle)
					+ position.x,
			(originalFrontRight.y - position.y) * std::cos(angle)
					+ (originalFrontRight.x - position.x) * std::sin(angle)
					+ position.y);

	return frontRight;
}

/**
 *
 */
Point Robot::getBackLeft() const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalBackLeft(x, y + size.y);

	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point backLeft(
			(originalBackLeft.x - position.x) * std::cos(angle)
					- (originalBackLeft.y - position.y) * std::sin(angle)
					+ position.x,
			(originalBackLeft.y - position.y) * std::cos(angle)
					+ (originalBackLeft.x - position.x) * std::sin(angle)
					+ position.y);

	return backLeft;

}

/**
 *
 */
Point Robot::getBackRight() const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalBackRight(x + size.x, y + size.y);

	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point backRight(
			(originalBackRight.x - position.x) * std::cos(angle)
					- (originalBackRight.y - position.y) * std::sin(angle)
					+ position.x,
			(originalBackRight.y - position.y) * std::cos(angle)
					+ (originalBackRight.x - position.x) * std::sin(angle)
					+ position.y);

	return backRight;
}

/**
 *
 */
Point Robot::getSafetyFrontLeft(unsigned short safetyMeasure) const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalFrontLeft(x - 2, y - safetyMeasure);
	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point frontLeft(
			(originalFrontLeft.x - position.x) * std::cos(angle)
					- (originalFrontLeft.y - position.y) * std::sin(angle)
					+ position.x,
			(originalFrontLeft.y - position.y) * std::cos(angle)
					+ (originalFrontLeft.x - position.x) * std::sin(angle)
					+ position.y);

	return frontLeft;
}

/**
 *
 */
Point Robot::getSafetyFrontRight(unsigned short safetyMeasure) const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalFrontRight(x + size.x + 2, y - safetyMeasure);
	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point frontRight(
			(originalFrontRight.x - position.x) * std::cos(angle)
					- (originalFrontRight.y - position.y) * std::sin(angle)
					+ position.x,
			(originalFrontRight.y - position.y) * std::cos(angle)
					+ (originalFrontRight.x - position.x) * std::sin(angle)
					+ position.y);

	return frontRight;
}

/**
 *
 */
Point Robot::getSafetyBackLeft(unsigned short safetyMeasure) const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalBackLeft(x - safetyMeasure, y + size.y + safetyMeasure);

	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point backLeft(
			(originalBackLeft.x - position.x) * std::cos(angle)
					- (originalBackLeft.y - position.y) * std::sin(angle)
					+ position.x,
			(originalBackLeft.y - position.y) * std::cos(angle)
					+ (originalBackLeft.x - position.x) * std::sin(angle)
					+ position.y);

	return backLeft;

}

/**
 *
 */
Point Robot::getSafetyBackRight(unsigned short safetyMeasure) const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalBackRight(x + size.x + safetyMeasure,
			y + size.y + safetyMeasure);

	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point backRight(
			(originalBackRight.x - position.x) * std::cos(angle)
					- (originalBackRight.y - position.y) * std::sin(angle)
					+ position.x,
			(originalBackRight.y - position.y) * std::cos(angle)
					+ (originalBackRight.x - position.x) * std::sin(angle)
					+ position.y);

	return backRight;
}

/**
 *
 */
void Robot::handleNotification() {
	//	std::unique_lock<std::recursive_mutex> lock(robotMutex);

	static int update = 0;
	if ((++update % 200) == 0) {
		notifyObservers();
	}
}

/**
 *
 */
void Robot::handleRequest(Messaging::Message& aMessage) {
	switch (aMessage.getMessageType()) {
	case EchoRequest: {
		Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": EchoRequest"));

		aMessage.setMessageType(EchoResponse);
		aMessage.setBody(": case 1 " + aMessage.asString());
		break;
	}
	case RequestWorld: {
		Application::Logger::log(
				__PRETTY_FUNCTION__
						+ std::string(": Ik ga dit request nu verwerken."));

		parseWorld(aMessage.getBody());

		aMessage.setMessageType(RequestWorld);
		//TO COMPILE THIS WAS DELETED
		aMessage.setBody(
				getRobotData() + "&" + getGoalData() + "&"
						+ RobotWorld::getRobotWorld().getWallData());
		/*
		 Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot( "Robot");
		 if (robot)
		 {
		 robot->sync(robot);
		 }
		 */
		break;
	}

	case SendRobotLocation: {
		updateAlienRobot(aMessage.getBody());

		Application::Logger::log(
				__PRETTY_FUNCTION__
						+ std::string(": De robot bevindt zich hier: .")
						+ aMessage.asString());

		aMessage.setMessageType(SendRobotLocation);
		aMessage.setBody("OK");
		break;
	}

	case SendStopMessage: {
		Application::Logger::log("De andere robot stop nu ook .");
		//driving = false;
		recalc = true;
		aMessage.setMessageType(SendStopMessage);

		//aMessage.setMessageType(SendRobotLocation);
		//aMessage.setBody("OK");
		break;
	}
	default: {
		Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": default"));

		aMessage.setBody(" default  Goodbye cruel world!");
		break;
	}
	}
}

/**
 *
 */
void Robot::handleResponse(const Messaging::Message& aMessage) {
	switch (aMessage.getMessageType()) {
	case EchoResponse: {
		Application::Logger::log(
				__PRETTY_FUNCTION__
						+ std::string(": case EchoResponse: not implemented, ")
						+ aMessage.asString());

		break;
	}

	case RequestWorld: {
		parseWorld(aMessage.getBody());
		Application::Logger::log(
				__PRETTY_FUNCTION__
						+ std::string(
								": De response is op dit adres aangekomen. ")
						+ aMessage.getBody());
		break;
	}
	case SendRobotLocation: {
		Application::Logger::log(
				__PRETTY_FUNCTION__
						+ std::string(": De robotdata is goed overgestuurd: ")
						+ aMessage.getBody());
		break;
	}
	case SendStopMessage: {
		break;
	}

	default: {
		Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": default not implemented, ")
						+ aMessage.asString());
		break;
	}
	}
}

/**
 *
 */
std::string Robot::asString() const {
	std::ostringstream os;

	os << "Robot " << name << " at (" << position.x << "," << position.y << ")";

	return os.str();
}

/**
 *
 */
std::string Robot::asDebugString() const {
	std::ostringstream os;

	os << "Robot:\n";
	os << AbstractAgent::asDebugString();
	os << "Robot " << name << " at (" << position.x << "," << position.y
			<< ")\n";

	return os.str();
}

/**
 *
 */
void Robot::drive() {
	try {

		if (speed == 0.0) {
			speed = 10.0;
		}

		unsigned pathPoint = 0;
		while (position.x > 0 && position.x < 500 && position.y > 0
				&& position.y < 500 && pathPoint < path.size()) {

			if (robotCollision()) {
				recalc = true;
				sendStopMessage();

				Application::Logger::log("Stop de robot");
			}

			if (recalc) {
				Application::Logger::log("recalculating route");
				speed = 0.0;
				if (Application::MainApplication::isArgGiven("-robot_type")
						&& Application::MainApplication::getArg("-robot_type").value
								== "client") {
					recalcRoute();
					recalc = false;
					speed = 10.0;
					pathPoint = 0;
					Application::Logger::log("Triggered robot being handled");
				} else {
					std::this_thread::sleep_for(
							std::chrono::milliseconds(3000));
					speed = 10.0;
					recalc = false;
					Application::Logger::log(
							"non-Triggered robot being handled");
				}
				notifyObservers();
			}

			if (!recalc) {
				const PathAlgorithm::Vertex& vertex = path[pathPoint += speed];
				front = BoundedVector(vertex.asPoint(), position);
				position.x = vertex.x;
				position.y = vertex.y;
			}

			if (arrived(goal) || collision()) {
				Application::Logger::log(
						__PRETTY_FUNCTION__
								+ std::string(": arrived or collision"));

				sendLocation();
				notifyObservers();
				break;
			}

			notifyObservers();

			sendLocation();

			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			// this should be the last thing in the loop
			if (driving == false) {
				return;
			}

		} // while

	} catch (std::exception& e) {
		std::cerr << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
	} catch (...) {
		std::cerr << __PRETTY_FUNCTION__ << ": unknown exception" << std::endl;
	}
}

/**
 *
 */
void Robot::calculateRoute(GoalPtr aGoal) {
	path.clear();
	if (aGoal) {
		// Turn off logging if not debugging AStar
		Application::Logger::setDisable();

		front = BoundedVector(aGoal->getPosition(), position);
		handleNotificationsFor(astar);
		path = astar.search(position, aGoal->getPosition(), size);
		stopHandlingNotificationsFor(astar);

		Application::Logger::setDisable(false);
	}
}

/**
 *
 */
bool Robot::arrived(GoalPtr aGoal) {
	if (aGoal && intersects(aGoal->getRegion())) {
		return true;
	}
	return false;
}

/**
 *
 */
bool Robot::collision() {
	Point frontLeft = getFrontLeft();
	Point frontRight = getFrontRight();
	Point backLeft = getBackLeft();
	Point backRight = getBackRight();

	const std::vector<WallPtr>& walls = RobotWorld::getRobotWorld().getWalls();
	for (WallPtr wall : walls) {
		if (Utils::Shape2DUtils::intersect(frontLeft, frontRight,
				wall->getPoint1(), wall->getPoint2())
				|| Utils::Shape2DUtils::intersect(frontLeft, backLeft,
						wall->getPoint1(), wall->getPoint2())
				|| Utils::Shape2DUtils::intersect(frontRight, backRight,
						wall->getPoint1(), wall->getPoint2())) {
			return true;
		}
	}
	return false;
}

bool Robot::robotCollision() {
	unsigned short safetyMeasure = 80;
	RobotPtr robot = RobotWorld::getRobotWorld().getRobot("Robot");
	RobotPtr robo2 = RobotWorld::getRobotWorld().getRobot("Robo2");
	Point robotPoly[] = { robot->getSafetyFrontLeft(safetyMeasure),
			robot->getSafetyFrontRight(safetyMeasure), robot->getFrontLeft(),
			robot->getFrontRight() };

	if (robo2 != nullptr) {
		if (Utils::Shape2DUtils::isInsidePolygon(robotPoly, 4,
				robo2->getFrontLeft())
				|| Utils::Shape2DUtils::isInsidePolygon(robotPoly, 4,
						robo2->getFrontRight())
				|| Utils::Shape2DUtils::isInsidePolygon(robotPoly, 4,
						robo2->getBackLeft())
				|| Utils::Shape2DUtils::isInsidePolygon(robotPoly, 4,
						robo2->getBackRight())) {
			Application::Logger::log(std::string("Robot detected to close"));
			return true;
		}
	}
	return false;
}

std::string Robot::getRobotData() const {
	std::ostringstream os;
	os << std::to_string(position.x) << "," << std::to_string(position.y) << ","
			<< std::to_string(front.x) << "," << std::to_string(front.y) << ","
			<< std::to_string(speed);
	return os.str();
}

std::string Robot::getGoalData() const {
	std::ostringstream os;
	GoalPtr goal = RobotWorld::getRobotWorld().getGoal("Goal");
	if (goal != nullptr) {
		os << std::to_string(goal->getPosition().x) << ","
				<< std::to_string(goal->getPosition().y);
	}
	return os.str();
}

void Robot::sendLocation() {
	Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot(
			"Robot");
	std::string remoteIpAdres = "localhost";
	std::string remotePort = "12346";

	if (Application::MainApplication::isArgGiven("-robot_type")) {
		if (Application::MainApplication::getArg("-robot_type").value
				== "client") {
			remotePort = "12345";
		}
	}
	if (Application::MainApplication::isArgGiven("-remote_ip")) {
		remoteIpAdres =
				Application::MainApplication::getArg("-remote_ip").value;
	}

	// We will request an echo message. The response will be "Hello World", if all goes OK,
	// "Goodbye cruel world!" if something went wrong.
	Messaging::Client c1ient(remoteIpAdres, remotePort, robot);
	Messaging::Message message(Model::Robot::MessageType::SendRobotLocation,
			getRobotData());
	c1ient.dispatchMessage(message);
}

void Robot::sendStopMessage() {
	Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot(
			"Robot");
	std::string remoteIpAdres = "localhost";
	std::string remotePort = "12346";

	if (Application::MainApplication::isArgGiven("-robot_type")) {
		if (Application::MainApplication::getArg("-robot_type").value
				== "client") {
			remotePort = "12345";
		}
	}
	if (Application::MainApplication::isArgGiven("-remote_ip")) {
		remoteIpAdres =
				Application::MainApplication::getArg("-remote_ip").value;
	}

	// We will request an echo message. The response will be "Hello World", if all goes OK,
	// "Goodbye cruel world!" if something went wrong.
	Messaging::Client c1ient(remoteIpAdres, remotePort, robot);
	Messaging::Message message(Model::Robot::MessageType::SendStopMessage);
	c1ient.dispatchMessage(message);
}

void Robot::recalcRoute() {
	std::vector<WallPtr> tempWalls;

	Application::Logger::log("Recalc function");

	RobotPtr robo2 = RobotWorld::getRobotWorld().getRobot("Robo2");

	tempWalls.push_back(
			RobotWorld::getRobotWorld().newWall(robo2->getFrontLeft(),
					robo2->getFrontRight(), false));
	tempWalls.push_back(
			RobotWorld::getRobotWorld().newWall(robo2->getFrontRight(),
					robo2->getBackRight(), false));
	tempWalls.push_back(
			RobotWorld::getRobotWorld().newWall(robo2->getBackRight(),
					robo2->getBackLeft(), false));
	tempWalls.push_back(
			RobotWorld::getRobotWorld().newWall(robo2->getBackLeft(),
					robo2->getFrontLeft(), false));

	//Recalc route
	goal = RobotWorld::getRobotWorld().getGoal("Goal");
	calculateRoute(goal);

	Application::Logger::log("Start driving has been called");

	//remove walls
	while (!tempWalls.empty()) {
		RobotWorld::getRobotWorld().deleteWall(tempWalls.at(0));
		tempWalls.erase(tempWalls.begin());
	}

	Application::Logger::log("End of recalc");
}

void Robot::parseWorld(const std::string& message) {
	std::vector<std::string> data;

	data = tokeniseString(message, '&');

	createAlienRobot(data.at(0));
	createAlienGoal(data.at(1));
	createAlienWalls(data.at(2));

	//Once everyone is added it's time to update the view
	notifyObservers();
}

void Robot::createAlienRobot(const std::string& message) {
	std::vector<std::string> data;

	data = tokeniseString(message, ',');

	if (RobotWorld::getRobotWorld().getRobot("Robo2") == nullptr) {
		RobotWorld::getRobotWorld().newRobot("Robo2",
				Point(stoi(data.at(0)), stoi(data.at(1))), false);
		Application::Logger::log("Make alien robot");
	} else {
		Application::Logger::log("Robot already exists");
	}
}

void Robot::createAlienWalls(const std::string& message) {
	std::vector<std::string> data;

	data = tokeniseString(message, ';');

	for (const std::string& d : data) {
		std::vector<std::string> walls;

		walls = tokeniseString(d, ',');
		RobotWorld::getRobotWorld().newWall(
				Point(stoi(walls.at(0)), stoi(walls.at(1))),
				Point(stoi(walls.at(2)), stoi(walls.at(3))), false);
	}
}

void Robot::createAlienGoal(const std::string& message) {
	std::vector<std::string> data = tokeniseString(message, ',');

	if (RobotWorld::getRobotWorld().getGoal("Goa2") == nullptr) {
		RobotWorld::getRobotWorld().newGoal("Goa2",
				Point(stoi(data.at(0)), stoi(data.at(1))), false);
	} else {
		Application::Logger::log("Goal already exists");
	}
}

void Robot::updateAlienRobot(const std::string& message) {
	std::vector<std::string> data = tokeniseString(message, ',');

	RobotPtr alien = RobotWorld::getRobotWorld().getRobot("Robo2");

	if (alien) {
		alien->setPosition(Point(stoi(data.at(0)), stoi(data.at(1))), true);
		alien->setFront(BoundedVector(stof(data.at(2)), stof(data.at(3))),
				true);
	}

	notifyObservers();
}

std::vector<std::string> Robot::tokeniseString(const std::string& message,
		char seperator) {
	std::vector<std::string> data;
	std::stringstream ss;
	ss.str(message);
	std::string item;
	while (std::getline(ss, item, seperator)) {
		data.push_back(item);
		//Application::Logger::log("Parsed item: " +item);
	}
	return data;
}

void Robot::sync(Model::RobotPtr robot) {
	std::string remoteIpAdres = "localhost";
	std::string remotePort = "12346";

	if (Application::MainApplication::isArgGiven("-robot_type")) {
		remotePort = "12345";
	}
	if (Application::MainApplication::isArgGiven("-remote_ip")) {
		remoteIpAdres =
				Application::MainApplication::getArg("-remote_ip").value;
	}

	Messaging::Client c1ient(remoteIpAdres, remotePort, robot);

	Messaging::Message message;

	//TO COMPILE THIS WAS DELETED
	message.setBody(
			getRobotData() + "&" + getGoalData() + "&"
					+ RobotWorld::getRobotWorld().getWallData());
	message.setMessageType(RequestWorld);
	c1ient.dispatchMessage(message);
}
} // namespace Model
