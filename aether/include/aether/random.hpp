#ifndef _AETHER_RANDOM_HPP_
#define _AETHER_RANDOM_HPP_
/*
   Based on a C-program for MT19937, with initialization improved 2002/1/26.
   Coded by Takuji Nishimura and Makoto Matsumoto.

   Before using, initialize the state by using init_genrand(seed)
   or init_by_array(init_key, key_length).

   Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.

     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

     3. The names of its contributors may not be used to endorse or promote
        products derived from this software without specific prior written
        permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cmath>
#include <cstdint>

#include <aether/types.hpp>

// Custom implementation, because <random> includes <string> for some reason
class RandomGenerator {
public:
    RandomGenerator() : RandomGenerator(42) {}

    RandomGenerator(uint32_t seed) { init_genrand(seed); }

    /*
     * Generates a random integer in the range [0, 0xffffffff]
     */
    uint32_t get_rand_int() {
        uint32_t y;

        y = mt[mti++];
        if (mti >= N) {
            mti = 0;
        }

        /* Tempering */
        y ^= (y >> 11);
        y ^= (y << 7) & 0x9d2c5680UL;
        y ^= (y << 15) & 0xefc60000UL;
        y ^= (y >> 18);

        return y;
    }

    /*
     * Generates a random float in the range [0, 1)
     */
    float get_rand_float() {
        return get_rand_int() / static_cast<float>(0xffffffffUL);
    }

    float uniform(float a, float b) { return a + (b - a) * get_rand_float(); }

    float normal(float mean, float std) {
        // Box-Muller transform
        float u1 = get_rand_float();
        float u2 = get_rand_float();
        return mean + std * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI_F * u2);
    }

    void regenerate() {
        // generate N words at one time
        static uint32_t mag01[2] = {0x0UL, MATRIX_A};
        // mag01[x] = x * MATRIX_A  for x=0,1
        uint32_t y;
        uint32_t kk;

        for (kk = 0; kk < N - M; kk++) {
            y = (mt[kk] & UPPER_MASK) | (mt[kk + 1] & LOWER_MASK);
            mt[kk] = mt[kk + M] ^ (y >> 1) ^ mag01[y & 0x1UL];
        }
        for (; kk < N - 1; kk++) {
            y = (mt[kk] & UPPER_MASK) | (mt[kk + 1] & LOWER_MASK);
            mt[kk] = mt[kk + (M - N)] ^ (y >> 1) ^ mag01[y & 0x1UL];
        }
        y = (mt[N - 1] & UPPER_MASK) | (mt[0] & LOWER_MASK);
        mt[N - 1] = mt[M - 1] ^ (y >> 1) ^ mag01[y & 0x1UL];

        mti = 0;
    }

    float rand_left() { return 1.0f - mti / static_cast<float>(N); }

private:
    static constexpr uint32_t N = 624;
    static constexpr uint32_t M = 397;
    static constexpr uint32_t MATRIX_A = 0x9908b0dfUL;
    static constexpr uint32_t UPPER_MASK = 0x80000000UL;
    static constexpr uint32_t LOWER_MASK = 0x7fffffffUL;

    uint32_t mt[N];       // the array for the state vector
    uint32_t mti = N + 1; // mti==N+1 means mt[N] is not initialized

    void init_genrand(uint32_t s) {
        mt[0] = s;
        for (mti = 1; mti < N; mti++) {
            mt[mti] =
                (1812433253UL * (mt[mti - 1] ^ (mt[mti - 1] >> 30)) + mti);
            // See Knuth TAOCP Vol2. 3rd Ed. P.106 for multiplier.
            // In the previous versions, MSBs of the seed affect
            // only MSBs of the array mt[].
            // 2002/01/09 modified by Makoto Matsumoto
        }
    }
};

#endif
