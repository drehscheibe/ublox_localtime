// Melchior FRANZ <mfranz@aon.at> 2018-01-19
//
#include "fletcher.h"


void fl16_clr(struct fletcher16_t *f)
{
	f->a = f->b = 0;
}


void fl16_add(struct fletcher16_t *f, uint8_t c)
{
	f->a += c;
	f->b += f->a;
}


void fl16_adds(struct fletcher16_t *f, const void *data, int len)
{
	const uint8_t *d = (const uint8_t *)data;

	while (len--)
		fl16_add(f, *d++);
}


uint16_t fl16_sum(const struct fletcher16_t *f)
{
	return ((uint16_t)f->a << 8) | f->b;
}
