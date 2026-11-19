/* Checks that the IQ FIFO hands out every queued block once, in order, and loses no buffer.
 * Run with "make test".
 */
#include <stdio.h>
#include <unistd.h>
#include "fifo.h"

int main(void) {
    enum { N = 8 };
    struct iq_buf *b[N];
    int failures = 0;

    alarm(2); // A lost block leaves fifo_dequeue waiting forever: fail instead of hanging
    fifo_create(N, N, 1);
    for (int round = 0; round < 3; round++) { // Buffers must all come back for the next round
        for (int i = 0; i < N; i++) {
            if ((b[i] = fifo_acquire()) == NULL) {
                printf("FAIL round %d: buffer %d not available\n", round, i);
                return 1;
            }
            b[i]->validLength = i + 1; // Marks the block
            fifo_enqueue(b[i]);
        }
        for (int i = 0; i < N; i++) {
            struct iq_buf *got = fifo_dequeue();
            if (got != b[i] || got->validLength != (unsigned) i + 1) {
                printf("FAIL round %d: block %d out of order or lost\n", round, i);
                failures++;
            }
            fifo_release(got);
        }
    }
    fifo_destroy();
    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures != 0;
}
