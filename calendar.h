// Melchior FRANZ <mfranz@aon.at> 2018-01-19
//
#ifndef CALENDAR_H_
#define CALENDAR_H_

#include <stdbool.h>


enum {
	SUNDAY = 0,
	MONDAY,
	TUESDAY,
	WEDNESDAY,
	THURSDAY,
	FRIDAY,
	SATURDAY,
};


enum {
	JANUARY = 1,
	FEBRUARY,
	MARCH,
	APRIL,
	MAY,
	JUNE,
	JULY,
	AUGUST,
	SEPTEMBER,
	OCTOBER,
	NOVEMBER,
	DECEMBER,
};


bool is_leap_year(unsigned year);
unsigned month_days(unsigned day, unsigned month);                     // 28, 29, 30. 31
unsigned day_of_year(unsigned day, unsigned month, unsigned year);     // 1..366
unsigned weekday(unsigned day, unsigned month, unsigned year);         // 0..6
unsigned calendar_week(unsigned day, unsigned month, unsigned year);   // 1..53
bool is_dst(unsigned hour, unsigned day, unsigned month, unsigned year);

#endif // CALENDAR_H_
