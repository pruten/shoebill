/*
 * Copyright (c) 2013, Peter Rutenbar <pruten@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// FIXME: unify all these weird instruction decoders (cpu/68881/68851)

#ifndef _MC68851_H
#define _MC68851_H

#include <stdint.h>

void inst_mc68851_prestore();
void inst_mc68851_psave();
void inst_mc68851_pbcc();

void inst_mc68851_pdbcc(uint16_t cond);
void inst_mc68851_ptrapcc(uint16_t cond);
void inst_mc68851_pscc(uint16_t cond);

void inst_mc68851_pload(uint16_t ext);
void inst_mc68851_pvalid(uint16_t ext);
void inst_mc68851_pflush(uint16_t ext);
void inst_mc68851_pmove(uint16_t ext);
void inst_mc68851_ptest(uint16_t ext);
void inst_mc68851_pflushr(uint16_t ext);


void dis_mc68851_prestore();
void dis_mc68851_psave();
void dis_mc68851_pbcc();

void dis_mc68851_pdbcc(uint16_t cond);
void dis_mc68851_ptrapcc(uint16_t cond);
void dis_mc68851_pscc(uint16_t cond);

void dis_mc68851_pload(uint16_t ext);
void dis_mc68851_pvalid(uint16_t ext);
void dis_mc68851_pflush(uint16_t ext);
void dis_mc68851_pmove(uint16_t ext);
void dis_mc68851_ptest(uint16_t ext);
void dis_mc68851_pflushr(uint16_t ext);

#endif // _MC68851_H