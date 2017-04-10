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
#ifndef TIMINGTOOLS_HPP_
#define TIMINGTOOLS_HPP_
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>

namespace labust
{
  namespace tools
  {
    /**
     * The function to calculate the UNIX time.
     *
     * \return Number of decimal seconds since 01.01.1970.
     */
    inline double unix_time()
    {
      using namespace boost::posix_time;
      using namespace boost::gregorian;
      ptime epoch(date(1970,1,1));
      ptime t(microsec_clock::local_time());
      return (t-epoch).total_microseconds()/1000000.;
    }
    /**
     * The function generates a string format time signatures. Useful
     * for creating time stamped filenames.
     *
     * \return String based time signature.
     */
    inline std::string time_signature()
    {
      using namespace boost::posix_time;
      using namespace boost::gregorian;
      std::stringstream out;
      ptime t(second_clock::local_time());
      out<<t.date().year();
      out<<"_"<<t.date().month();
      out<<"_"<<t.date().day();
      out<<"_"<<t.time_of_day().hours();
      out<<"_"<<t.time_of_day().minutes();
      out<<"_"<<t.time_of_day().seconds();

      return out.str();
    }
    /**
     * The function returns the unix_time in a string format.
     *
     * \param microseconds True if we wish to display the microseconds.
     * \return UNIX time in string format.
     */
    inline std::string unix_time_string(bool microseconds = false)
    {
      std::stringstream out;
      out.precision(6);
      out<<std::fixed<<(microseconds ? unix_time() : long(unix_time()));
      return out.str();
    }
    /**
     * The function suspends the system for the specified amount of milliseconds.
     *
     * \param ms The amount of milliseconds to sleep.
     */
    inline void sleep(size_t ms)
    {
    	boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
    };
    /**
     * The structure creates a delay timer that waits until a millisecond multiple.
     * Useful when precise timing is required since it takes into account the amount
     * of time passed between calls.
     */
    struct wait_until_ms
    {
    	/**
    	 * Main constructor. Sets the millisecond multiple.
    	 *
    	 * \param ms The delay time.
    	 */
      wait_until_ms(size_t ms):
#ifdef unix
      	time(ms*1000){};
#else
      	time(ms){};
#endif
      /**
       * The operator triggers the suspend time.
       *
       */
      inline void operator() ()
      {
        using namespace boost;
        using namespace posix_time;
#ifdef unix
        this_thread::sleep(microseconds(time - microsec_clock::local_time().time_of_day().total_microseconds()%time));
#else
        this_thread::sleep(milliseconds(time - microsec_clock::local_time().time_of_day().total_milliseconds()%time));
#endif
      }
	private:
      /**
       * Amount of microseconds to sleep.
       */
      size_t time;
    };
    /**
     * This structure is used to perform watchdog operations. The watchdog timer triggers
     * the user defined function.
     */
    struct watchdog : boost::noncopyable
    {
    	/**
    	 * Main constructor. Takes the user desired function and timing.
    	 *
    	 * \param func The user defined functor to be called when the watchdog triggers.
    	 * \param timeout_ms The timeout time after which to trigger.
    	 * \param loop_ms The loop time of the watchdog function.
    	 */
      watchdog(const boost::function<void (void)>& func, size_t timeout_ms = 500, size_t loop_ms = 100):
        func(func),
        counter(0),
        timeout_ms(timeout_ms),
        loop_ms(loop_ms),
        runner(boost::bind(&watchdog::run,this)){};
      /**
       * Generic destructor.
       */
      ~watchdog()
      {
        runner.interrupt();
        runner.join();
      }

      /**
       * The method resets the watchdog counter.
       */
      void reset(){boost::mutex::scoped_lock lock(cntMutex); this->counter = 0;};

    private:
      /**
       * The method performs the watchdog operation.
       */
      void run()
      try
      {
      	while (true)
      	{
      		using namespace boost;
      		using namespace posix_time;
      		sleep(loop_ms);
      		boost::mutex::scoped_lock lock(cntMutex);
      		counter += loop_ms;
      		if (counter >= timeout_ms) func();
      	}
      }
      catch (boost::thread_interrupted&){};

      /**
       * The user functor.
       */
      boost::function<void (void)> func;
      /**
       * The counter variable.
       */
      volatile size_t counter;
      /**
       * The watchdog timeout and loop time.
       */
      size_t timeout_ms, loop_ms;
      /**
       * The watchdog execution thread.
       */
      boost::thread runner;
      /**
       * The protector mutex for access synchonization to the counter variable.
       */
      boost::mutex cntMutex;
    };
  }
}
/* TIMINGTOOLS_HPP_ */
#endif
