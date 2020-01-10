import unittest

from optimization import adapt, adapt_bruteforce
from missions import (
    generate_simple_3D_reconstruction,
    generate_batt_messages,
    simulate_sending_messages_with_latency,
)


class BasicTests(unittest.TestCase):
    def test_basic_two_agents(self):
        messages, ctx = generate_simple_3D_reconstruction(8)
        simulate_sending_messages_with_latency(messages, 0.01)

        def available_throughput(t):
            return 1

        timeslot_length = 1
        result1 = adapt(
            messages, ctx, available_throughput, timeslot_length
        )
        result2 = adapt_bruteforce(
            messages, ctx, available_throughput, timeslot_length
        )
        self.assertEqual(ctx.utility(result1), ctx.utility(result2))

    def test_long_utilitiy_messages(self):
        messages1, ctx1 = generate_batt_messages(
            4, {0}, {1}, level_end=0.9999
        )
        messages2, ctx2 = generate_batt_messages(
            4, {1}, {0}, level_start=0.1, level_end=0.001
        )
        messages = messages1 + messages2
        ctx = ctx1
        simulate_sending_messages_with_latency(messages, 0.01)

        def available_throughput(t):
            return 1

        timeslot_length = 1
        result1 = adapt(
            messages, ctx, available_throughput, timeslot_length
        )
        result2 = adapt_bruteforce(
            messages, ctx, available_throughput, timeslot_length
        )
        print(result1)
        print(result2)
        self.assertEqual(ctx.utility(result1), ctx.utility(result2))


if __name__ == '__main__':
    unittest.main()
