/* License and attribution: LICENSES/source_notices/notice_030.txt */




#pragma once

#ifdef _AVR_
#error there is no implementation for waiter.h on AVR platforms
#else

#include "hal/event.h"

namespace rp{ namespace hal{

	template<typename ResultT>
	class Waiter : public Event
	{
	public:
		Waiter() 
			: Event()
		{
		}

		~Waiter() 
		{}

		ResultT waitForResult()
		{
			wait();
			return result;
		}

		void setResult(ResultT result)
		{
			this->result = result;
			set();
		}

		volatile ResultT result;
	};
}}

#endif
