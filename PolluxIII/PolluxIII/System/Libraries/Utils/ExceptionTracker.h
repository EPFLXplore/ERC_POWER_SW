/*
 * ExceptionTracker.h
 *
 *  Created on: Jun 9, 2023
 *      Author: arion
 */

#ifndef LIBRARIES_EMBEDDEDGL_EXCEPTIONHANDLER_H_
#define LIBRARIES_EMBEDDEDGL_EXCEPTIONHANDLER_H_

#include <cstdint>

#define TRACK_EXCEPTIONS() \
	virtual const char* __getSource() { \
		return __FILE__; \
	}

#define throwException(name) \
	__throwExc(name, __LINE__)


class ExceptionTracker {
public:
	void trackExceptions(ExceptionTracker* parent);

protected:
	~ExceptionTracker() {}
	virtual const char* __getSource() = 0;
	void __throwExc(const char* name, uint32_t line);

private:
	ExceptionTracker* parent;
};

#endif /* LIBRARIES_LIGHTWEIGHTEXCEPTIONS_EXCEPTIONTRACKER_H_ */
