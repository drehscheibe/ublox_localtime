// Melchior FRANZ <mfranz@aon.at> 2018-01-19
//
#include "fletcher.h"


void fl8_clr(struct fletcher8_t *f)
{
	f->a = f->b = 0;
}


void fl8_add(struct fletcher8_t *f, uint8_t c)
{
	f->a += c;
	f->b += f->a;
}


void fl8_adds(struct fletcher8_t *f, const void *data, int len)
{
	const uint8_t *d = (const uint8_t *)data;

	while (len--)
		fl8_add(f, *d++);
}


uint16_t fl8_sum(const struct fletcher8_t *f)
{
	return ((uint16_t)f->a << 8) | f->b;
}
