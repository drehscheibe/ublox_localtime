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

// return whether a given UTC-hour/day/month/year is in DST according to EU rules;
// if switchday is non-NULL, it's set to true if the DST begins/ends on that day
bool is_dst(unsigned hour, unsigned day, unsigned month, unsigned year,
		bool *switchday);

#endif // CALENDAR_H_
