// Melchior FRANZ <mfranz@aon.at> 2018-01-19
//
#include <assert.h>

#include "calendar.h"


bool is_leap_year(unsigned year)
{
	if (!(year % 400))
		return true;

	if (!(year % 100))
		return false;

	if (!(year % 4))
		return true;

	return false;
}


unsigned month_days(unsigned month, unsigned year)
{
	assert(month >= JANUARY && month <= DECEMBER);
	static const char mdays[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	unsigned days = mdays[month - JANUARY];
	if (month == FEBRUARY && is_leap_year(year))
		days++;
	return days;
}


unsigned day_of_year(unsigned day, unsigned month, unsigned year) // 1..366
{
	assert(month >= JANUARY && month <= DECEMBER);
	static const unsigned mdays[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
	unsigned days = mdays[month - JANUARY] + day;
	if (month > FEBRUARY && is_leap_year(year))
		days++;
	return days;
}


unsigned weekday(unsigned day, unsigned month, unsigned year) // 0..6  ->  sunday..saturday
{
	unsigned y = year - 1;
	unsigned days = y * 365 + y / 4 - y / 100 + y / 400
			+ day_of_year(day, month, year);
	return days % 7;
}


unsigned calendar_week(unsigned day, unsigned month, unsigned year) // 1..53
{
	int first_weekday = weekday(1, JANUARY, year);
	bool leap = is_leap_year(year);
	int last_weekday = (first_weekday + leap) % 7;
	int last_sunday = 365 + leap - last_weekday + (last_weekday >= THURSDAY) * 7;

	int doy = day_of_year(day, month, year);
	if (doy > last_sunday)
		return 1; // belongs to the next year already

	int first_monday = (11 - first_weekday) % 7 - 2;
	if (doy >= first_monday)
		return 1 + (doy - first_monday) / 7;

	// belongs to the previous year
	if (first_weekday == FRIDAY)
		return 53;

	if (first_weekday == SUNDAY)
		return 52;

	// first_weekday == SATURDAY
	return is_leap_year(year - 1) ? 53 : 52;
}


bool is_dst(unsigned hour, unsigned day, unsigned month, unsigned year,
		bool *switchday) // UTC input, EU rules
{
	if (switchday)
		*switchday = false;

	if (month > MARCH && month < OCTOBER)
		return true;

	if (month < MARCH || month > OCTOBER)
		return false;

	bool is_march = month == MARCH; // else OCTOBER
	int last_sunday = 31 - weekday(31, month, year);

	if (day < last_sunday)
		return !is_march;

	if (day > last_sunday)
		return is_march;

	if (switchday)
		*switchday = true;

	return is_march ^ !hour;
}
