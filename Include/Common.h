#ifndef COMMON_H
#define COMMON_H
#include <stdio.h>
#include <string.h>
#include "Recast.h"
#include "RecastDump.h"
#include "PerfTimer.h"

//#define SAFE_DELETE(a) if( (a) != NULL ) delete (a); (a) = NULL;

/// These are just sample areas to use consistent values across the samples.
/// The use should specify these base on his needs.
enum PolyAreasType
{
	POLYAREA_GROUND,
	POLYAREA_WATER,
	POLYAREA_ROAD,
	POLYAREA_DOOR,
	POLYAREA_GRASS,
	POLYAREA_JUMP,
};

enum PolyFlags
{
	POLYFLAGS_WALK		= 0x01,		// Ability to walk (ground, grass, road)
	POLYFLAGS_SWIM		= 0x02,		// Ability to swim (water).
	POLYFLAGS_DOOR		= 0x04,		// Ability to move through doors.
	POLYFLAGS_JUMP		= 0x08,		// Ability to jump.
	POLYFLAGS_DISABLED	= 0x10,		// Disabled polygon
	POLYFLAGS_ALL		= 0xffff	// All abilities.
};

enum PartitionType
{
	PARTITION_WATERSHED,
	PARTITION_MONOTONE,
	PARTITION_LAYERS,
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Uncomment this to dump all the requests in stdout.
//#define DUMP_REQS

// These are example implementations of various interfaces used in Recast and Detour.

/// Recast build context.
class BuildContext : public rcContext
{
	TimeVal m_startTime[RC_MAX_TIMERS];
	TimeVal m_accTime[RC_MAX_TIMERS];

	static const int MAX_MESSAGES = 1000;
	const char* m_messages[MAX_MESSAGES];
	int m_messageCount;
	static const int TEXT_POOL_SIZE = 8000;
	char m_textPool[TEXT_POOL_SIZE];
	int m_textPoolSize;
	
public:
	BuildContext();
	
	/// Dumps the log to stdout.
	void dumpLog(const char* format, ...);
	/// Returns number of log messages.
	int getLogCount() const;
	/// Returns log message text.
	const char* getLogText(const int i) const;
	
protected:	
	/// Virtual functions for custom implementations.
	///@{
	virtual void doResetLog();
	virtual void doLog(const rcLogCategory category, const char* msg, const int len);
	virtual void doResetTimers();
	virtual void doStartTimer(const rcTimerLabel label);
	virtual void doStopTimer(const rcTimerLabel label);
	virtual int doGetAccumulatedTime(const rcTimerLabel label) const;
	///@}
};

/// stdio file implementation.
class FileIO : public duFileIO
{
	FILE* m_fp;
	int m_mode;
public:
	FileIO();
	virtual ~FileIO();
	bool openForWrite(const char* path);
	bool openForRead(const char* path);
	virtual bool isWriting() const;
	virtual bool isReading() const;
	virtual bool write(const void* ptr, const size_t size);
	virtual bool read(void* ptr, const size_t size);
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	FileIO(const FileIO&);
	FileIO& operator=(const FileIO&);
};





#endif
