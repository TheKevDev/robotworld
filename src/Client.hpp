#ifndef CLIENT_HPP_
#define CLIENT_HPP_

#include "Config.hpp"

#include <string>

#include "Session.hpp"

namespace Messaging
{
	class Client
	{
		public:
			/**
			 *
			 */
			Client( const std::string& aHost,
					const std::string& aPort,
					ResponseHandlerPtr aResponseHandler) :
							io_service( CommunicationService::getCommunicationService().getIOService()),
							host( aHost),
							port( aPort),
							responseHandler( aResponseHandler)
			{
			}
			/**
			 *
			 */
			~Client()
			{
			}
			/**
			 *
			 */
			void dispatchMessage( Message& aMessage)
			{
				// Create the session that will handle the next outgoing connection
				ClientSession* session = new ClientSession( aMessage, io_service, responseHandler);

				// Build up the remote address to which we will connect
				boost::asio::ip::tcp::resolver resolver( io_service);
				boost::asio::ip::tcp::resolver::query query( boost::asio::ip::tcp::v4(), host, port);
				boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve( query);
				boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;

				// Let the session handle any outgoing messages
				session->getSocket().async_connect( endpoint,
													boost::bind( &Client::handleConnect, this, session, boost::asio::placeholders::error));
			}
			/**
			 *
			 */
			void handleConnect( ClientSession* aSession,
								const boost::system::error_code& error)
			{
				if (!error)
				{
					aSession->start();
				} else
				{
					std::string sessionDestination =  "host = " + host + ", port =  "+ port;
					delete aSession;
					throw std::runtime_error( __PRETTY_FUNCTION__ + std::string( ": ") + error.message() + ", " + sessionDestination);
				}
			}

		private:
			boost::asio::io_service& io_service;
			std::string host;
			std::string port;
			ResponseHandlerPtr responseHandler;
	};
} // namespace Messaging

#endif // CLIENT_HPP_
