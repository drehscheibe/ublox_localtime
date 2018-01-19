// Melchior FRANZ <mfranz@aon.at> 2018-01-19
//
#ifndef FLETCHER_H_
#define FLETCHER_H_

#include <stdint.h>


struct fletcher8_t {
	uint8_t a;
	uint8_t b;
};


void fl8_clr(struct fletcher8_t *f);
void fl8_add(struct fletcher8_t *f, uint8_t c);
void fl8_adds(struct fletcher8_t *f, const void *data, int len);
uint16_t fl8_sum(const struct fletcher8_t *f);

#endif // FLETCHER_H_
