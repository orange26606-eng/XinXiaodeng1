#include "imu_terminal_gate.h"

#include <stdio.h>
#include <string.h>

static unsigned int checks;
static unsigned int failures;

#define CHECK(condition) do { \
    ++checks; \
    if (!(condition)) { \
        ++failures; \
        printf("FAIL line %d: %s\n", __LINE__, #condition); \
    } \
} while (0)

static int feed_text(struct imu_terminal_gate *gate, const char *text)
{
    return imu_terminal_gate_feed(gate, text, strlen(text));
}

static void fresh_prompt(struct imu_terminal_gate *gate)
{
    imu_terminal_gate_init(gate);
    imu_terminal_gate_prompt(gate);
}

static void test_initialization_and_early_input(void)
{
    struct imu_terminal_gate gate;

    memset(&gate, 0x55, sizeof(gate));
    imu_terminal_gate_init(&gate);
    CHECK(gate.prompted == 0);
    CHECK(gate.confirmed == 0);
    CHECK(gate.disconnected == 0);
    CHECK(gate.discard_line == 0);
    CHECK(gate.used == 0U);
    CHECK(gate.line[0] == '\0');
    CHECK(feed_text(&gate, "y\nY\r\n") == 0);
    CHECK(gate.confirmed == 0);
    CHECK(gate.used == 0U);
    imu_terminal_gate_prompt(&gate);
    CHECK(gate.prompted == 1);
    CHECK(gate.discard_line == 0);
    CHECK(feed_text(&gate, "y\n") == 1);
    CHECK(gate.confirmed == 1);

    imu_terminal_gate_init(&gate);
    CHECK(feed_text(&gate, "y") == 0);
    CHECK(gate.used == 1U);
    imu_terminal_gate_prompt(&gate);
    CHECK(gate.used == 0U);
    CHECK(gate.discard_line == 1);
    CHECK(feed_text(&gate, "\n") == 0);
    CHECK(gate.confirmed == 0);
    CHECK(gate.discard_line == 0);
    CHECK(feed_text(&gate, "Y\n") == 1);

    imu_terminal_gate_init(&gate);
    CHECK(feed_text(&gate, "Y\r") == 0);
    imu_terminal_gate_prompt(&gate);
    CHECK(feed_text(&gate, "\ny\n") == 1);
    CHECK(gate.confirmed == 1);

    imu_terminal_gate_init(&gate);
    CHECK(feed_text(&gate, "n\ny") == 0);
    imu_terminal_gate_prompt(&gate);
    imu_terminal_gate_prompt(&gate);
    CHECK(gate.discard_line == 1);
    CHECK(feed_text(&gate, "\r\n") == 0);
    CHECK(gate.confirmed == 0);
    CHECK(feed_text(&gate, "y\n") == 1);
}

static void test_partial_and_crlf(void)
{
    struct imu_terminal_gate gate;
    const char *valid[] = { "y\n", "Y\n", "y\r\n", "Y\r\n" };
    size_t variant;
    size_t split;

    for (variant = 0U; variant < sizeof(valid) / sizeof(valid[0]); ++variant) {
        size_t length = strlen(valid[variant]);
        for (split = 1U; split < length; ++split) {
            fresh_prompt(&gate);
            CHECK(imu_terminal_gate_feed(&gate, valid[variant], split) == 0);
            CHECK(gate.confirmed == 0);
            CHECK(imu_terminal_gate_feed(&gate, valid[variant] + split,
                                         length - split) == 1);
            CHECK(gate.confirmed == 1);
            CHECK(feed_text(&gate, "y\n") == 0);
        }
    }

    fresh_prompt(&gate);
    CHECK(feed_text(&gate, "y") == 0);
    CHECK(feed_text(&gate, "\r") == 0);
    CHECK(gate.confirmed == 0);
    CHECK(feed_text(&gate, "\n") == 1);

    fresh_prompt(&gate);
    CHECK(feed_text(&gate, "y\r") == 0);
    CHECK(gate.confirmed == 0);
    CHECK(feed_text(&gate, "\ry\n") == 0);
    CHECK(gate.confirmed == 0);
    CHECK(feed_text(&gate, "Y\r\n") == 1);
}

static void test_invalid_and_coalesced(void)
{
    struct imu_terminal_gate gate;
    const char *invalid[] = {
        "\n", "\r\n", "yes\n", "n\n", " y\n", "y \n",
        "y\t\n", "yy\n", "Yy\n", "y\r\r\n", "\ry\n"
    };
    const char binary_invalid[] = { 'y', '\0', '\n' };
    size_t index;

    fresh_prompt(&gate);
    for (index = 0U; index < sizeof(invalid) / sizeof(invalid[0]); ++index) {
        CHECK(feed_text(&gate, invalid[index]) == 0);
        CHECK(gate.confirmed == 0);
        CHECK(gate.used == 0U);
    }
    CHECK(imu_terminal_gate_feed(&gate, binary_invalid,
                                 sizeof(binary_invalid)) == 0);
    CHECK(gate.confirmed == 0);
    CHECK(feed_text(&gate, "invalid\nY\r\nextra\ny\n") == 1);
    CHECK(gate.confirmed == 1);
    CHECK(feed_text(&gate, "Y\n") == 0);
    imu_terminal_gate_prompt(&gate);
    CHECK(gate.confirmed == 1);

    fresh_prompt(&gate);
    CHECK(feed_text(&gate, "invalid\n") == 0);
    CHECK(feed_text(&gate, "y\ny\n") == 1);
    CHECK(feed_text(&gate, "\n") == 0);
}

static void test_overflow(void)
{
    struct imu_terminal_gate gate;
    char long_line[128];
    size_t index;

    memset(long_line, 'x', sizeof(long_line));
    fresh_prompt(&gate);
    CHECK(imu_terminal_gate_feed(&gate, long_line, 31U) == 0);
    CHECK(gate.used == 31U);
    CHECK(gate.discard_line == 0);
    CHECK(feed_text(&gate, "\ny\n") == 1);

    fresh_prompt(&gate);
    CHECK(imu_terminal_gate_feed(&gate, long_line, 32U) == 0);
    CHECK(gate.used == 0U);
    CHECK(gate.discard_line == 1);
    CHECK(feed_text(&gate, "y\n") == 0);
    CHECK(gate.confirmed == 0);
    CHECK(gate.discard_line == 0);
    CHECK(feed_text(&gate, "Y\n") == 1);

    imu_terminal_gate_init(&gate);
    CHECK(imu_terminal_gate_feed(&gate, long_line, sizeof(long_line)) == 0);
    CHECK(gate.discard_line == 1);
    imu_terminal_gate_prompt(&gate);
    CHECK(gate.discard_line == 1);
    CHECK(feed_text(&gate, "y\n") == 0);
    CHECK(gate.confirmed == 0);
    CHECK(feed_text(&gate, "y\r\n") == 1);

    fresh_prompt(&gate);
    for (index = 0U; index < sizeof(long_line); ++index) {
        CHECK(imu_terminal_gate_feed(&gate, long_line + index, 1U) == 0);
    }
    CHECK(gate.discard_line == 1);
    CHECK(feed_text(&gate, "\ny\n") == 1);
}

static void test_disconnect_and_empty_input(void)
{
    struct imu_terminal_gate gate;

    imu_terminal_gate_init(&gate);
    CHECK(imu_terminal_gate_disconnect(&gate) == 1);
    CHECK(gate.disconnected == 1);
    CHECK(gate.confirmed == 0);
    imu_terminal_gate_prompt(&gate);
    CHECK(gate.prompted == 0);
    CHECK(feed_text(&gate, "y\n") == 0);

    fresh_prompt(&gate);
    CHECK(feed_text(&gate, "y\r") == 0);
    CHECK(imu_terminal_gate_disconnect(&gate) == 1);
    CHECK(gate.confirmed == 0);
    CHECK(gate.used == 0U);
    CHECK(feed_text(&gate, "\nY\n") == 0);
    CHECK(gate.confirmed == 0);
    CHECK(imu_terminal_gate_disconnect(&gate) == 1);

    fresh_prompt(&gate);
    CHECK(imu_terminal_gate_feed(&gate, "y\n", 0U) == 0);
    CHECK(imu_terminal_gate_feed(&gate, NULL, 0U) == 0);
    CHECK(imu_terminal_gate_feed(&gate, NULL, 2U) == 0);
    CHECK(gate.confirmed == 0);
    CHECK(feed_text(&gate, "y\n") == 1);
    CHECK(imu_terminal_gate_disconnect(&gate) == 1);
    CHECK(gate.confirmed == 1);
    CHECK(gate.disconnected == 1);
    CHECK(feed_text(&gate, "y\n") == 0);

    imu_terminal_gate_init(NULL);
    imu_terminal_gate_prompt(NULL);
    CHECK(imu_terminal_gate_feed(NULL, "y\n", 2U) == 0);
    CHECK(imu_terminal_gate_disconnect(NULL) == 0);
}

int main(void)
{
    test_initialization_and_early_input();
    test_partial_and_crlf();
    test_invalid_and_coalesced();
    test_overflow();
    test_disconnect_and_empty_input();
    printf("Terminal gate: %u checks, %u failures\n", checks, failures);
    return failures == 0U ? 0 : 1;
}
