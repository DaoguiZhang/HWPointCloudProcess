#pragma once
#ifndef HW_SINGLETON
#define HW_SINGLETON

namespace HW
{
	//! Generic singleton encapsulation structure
	template<class T> struct HWSingleton
	{
		//! Default constructor
		HWSingleton() : instance(nullptr) {}
		//! Destructor
		~HWSingleton() { release(); }
		//! Releases the current instance
		inline void release() { if (instance) { delete instance; instance = nullptr; } }

		//! Current instance
		T* instance;
	};
}

#endif