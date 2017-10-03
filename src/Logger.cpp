#include "Logger.hpp"
#include "MainApplication.hpp"
#include "MainFrameWindow.hpp"
#include "DebugTraceFunction.hpp"

#include <chrono>
#include <ctime>

namespace Application {
/* static */bool Logger::disable = false;
/**
 *
 */
/*static*/void Logger::log(const std::string& aMessage) {
	Application::MainFrameWindow* frame =
			dynamic_cast<Application::MainFrameWindow*>(Application::TheApp().GetTopWindow());
	if (frame && !disable) {
		time_t rawtime;
		struct tm * timeinfo;
		char buffer[80];

		time(&rawtime);
		timeinfo = localtime(&rawtime);

		strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
		std::string str(buffer);
		frame->getTraceFunction().trace(str + ";" + aMessage);
	}
}
/**
 *
 */
/* static */void Logger::setDisable(bool aDisable /*= true*/) {
	disable = aDisable;
}
} //namespace Application
