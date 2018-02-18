// Melchior FRANZ <mfranz@aon.at> 2018-01-19
//
#ifndef FLETCHER_H_
#define FLETCHER_H_

#include <stdint.h>


struct fletcher16_t {
	uint8_t a;
	uint8_t b;
};


void fl16_clr(struct fletcher16_t *f);
void fl16_add(struct fletcher16_t *f, uint8_t c);
void fl16_adds(struct fletcher16_t *f, const void *data, int len);
uint16_t fl16_sum(const struct fletcher16_t *f);

#endif // FLETCHER_H_
