/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, LABUST, UNIZG-FER
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the LABUST nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#ifndef SENSORS_UTIL_ASCIISERIAL_H
#define SENSORS_UTIL_ASCIISERIAL_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <string>
#include <iosfwd>

namespace labust
{
	namespace comms
	{
		/**
		 * The class implements a generic ASCII serial protocol.
		 */
		class ASCIISerial
		{
			///The callback type definition
			typedef boost::function<void(const std::string&)> CallbackType;

		public:
			/**
			 * Main constructor
			 */
			ASCIISerial():
				port(io),
				connected(false){};
			/**
			 * Generic destructor.
			 */
			~ASCIISerial()
			{
				io.stop();
				runner.join();
			};

			/**
			 * Internal function for setting up the serial port.
			 * NOTE: The function will re-throw an exception on connection errors.
			 * @param portName Serial port name.
			 * @param baud Serial baud rate.
			 * @return True if connection is successful false otherwise
			 */
			bool connect(const std::string& port_name, int baud)
			try
			{
				using namespace boost::asio;
				port.open(port_name);
				port.set_option(serial_port::baud_rate(baud));
				port.set_option(serial_port::flow_control(
						serial_port::flow_control::none));

				connected = port.is_open();

				if (connected)
				{
					//Start the receive cycle
					this->startReceive();
					runner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
				}

				return connected;
			}
			catch (std::exception& e)
			{
				std::cerr<<e.what()<<std::endl;
				throw;
			}

			/**
			 * Register the message handler.
			 * @param callback The callback function that will be called on new data.
			 */
			void registerCallback(const CallbackType& callback)
			{
				boost::mutex::scoped_lock l(callback_mux);
				this->callback = callback;
			};
			/**
			 * Send a message to the serial port. NOTE: The message will not be modified, append any delimiters.
			 * @param msg The string will be forwarded directly to the serial port with a blocking call.
			 * @return True if message was sent successfully.
			 */
			inline bool send(const std::string& msg)
			{
				boost::asio::write(port, boost::asio::buffer(msg));
				return true;
			}

		private:
			/**
			 * Handle the incoming serial data stream.
			 * @param e	Serial error indicator
			 * @param size Size of the received message.
			 */
			void onData(const boost::system::error_code& e, std::size_t size)
			{
				if (!e)
				{
					std::istream is(&buffer);
					std::string data(size,'\0');
					is.read(&data[0],size);
					if (!callback.empty()) callback(data);
				}
				this->startReceive();
			}

			///Receive startup helper function.
			void startReceive()
			{
				boost::asio::async_read_until(port, buffer,
						"\r\n",
						boost::bind(&ASCIISerial::onData, this, _1,_2));
			}

			///Hardware i/o service.
			boost::asio::io_service io;
			///The serial input port
			boost::asio::serial_port port;
			///The main operation thread.
			boost::thread runner;
			///The input buffer.
			boost::asio::streambuf buffer;
			///Connection flag
			bool connected;

			///The message callback
			CallbackType callback;
			///Muxer for the callback
			boost::mutex callback_mux;
		};
	}
}

/* SENSORS_UTIL_ASCIISERIAL_H */
#endif
