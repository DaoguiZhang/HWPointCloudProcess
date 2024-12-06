#pragma once
//qt
#include<QString>

#include<QtCore/QtGlobal>
/*
This interface is meant to be used as a unique instance.
*/

class Q_DECL_EXPORT HWLog {
public:
	//!Destructor
	virtual ~HWLog();
	//!return the static and unique instance
	static HWLog* GetInstance();
	//!Register a unique instance
	static void RegisterInstance(HWLog* logInstance);
	//!Enables the message backup system
	static void EnableMessageBackup(bool state);

	//Message level
	enum MessageLevelFlags
	{
		LOG_STANDARD	= 0,
		LOG_DEBUG		= 1,
		LOG_WARNING		= 2,
		LOG_ERROR		= 4,
	};
	//Static shortcut to HWLog::logMessage
	static void LogMessage(const QString& message, int level);
	
	//! Generic message logging method
	/** To be implemented by child class.
	\warning MUST BE THREAD SAFE!
	\param message message
	\param level message severity (see MessageLevelFlags)
	**/
	virtual void logMessage(const QString& message, int level) = 0;
};