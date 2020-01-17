#! /usr/bin/env python
# -*- coding: utf-8 -*-
from itertools import chain, combinations
from collections import deque


from missions import (
    generate_simple_3D_reconstruction,
    simulate_sending_messages_with_latency,
)
from models import MessageSet


def powerset(iterable):
    "powerset([1,2,3]) --> () (1,) (2,) (3,) (1,2) (1,3) (2,3) (1,2,3)"
    s = list(iterable)
    return chain.from_iterable(combinations(s, r) for r in range(1, len(s)+1))


def used_throughput(messages, timeslot_length, i):
    j = 0
    while (
        i+j < len(messages.all()) and
        messages.all()[i+j].t_sent - messages.all()[i].t_sent <
        timeslot_length
    ):
        j += 1
    return j


def throughput_check(msgs, available_throughput, timeslot_length):
    for i, msg in enumerate(msgs.all()):
        if (
            used_throughput(msgs, timeslot_length, i) >
            available_throughput(msg.t_sent)
        ):
            return False
    return True


def adapt_bruteforce(messages, context, available_throughput, timeslot_length):

    all_subsets = list(powerset(messages.all()))
    all_subsets = [
        MessageSet(messages.t_end, list(msgs))
        for msgs in all_subsets
    ]
    all_subsets = [
        msgs
        for msgs in all_subsets
        if throughput_check(msgs, available_throughput, timeslot_length)
    ]
    best_set = max(
        all_subsets,
        key=lambda msgs: context.utility(msgs)
    )
    return best_set


def adapt_dynamic(messages, context, available_throughput, timeslot_length):
    possible_results = [
        MessageSet(messages.t_end, [], messages.t_end)
    ]
    window = deque()

    for m in messages.all()[::-1]:

        while window and (window[-1].t_sent - m.t_sent >= timeslot_length):
            window.pop()
            possible_results = [
                possible_results[i+1]
                if (
                    throughput_check(  # TODO: probably not needed
                        possible_results[i+1],
                        available_throughput,
                        timeslot_length
                    ) and (
                        context.utility(possible_results[i+1]) >
                        context.utility(possible_results[i])
                    )
                )
                else possible_results[i]
                for i in range(0, len(possible_results), 2)
            ]

        window.appendleft(m)

        results_with_m = [
            MessageSet(messages.t_end, [m], m.t_sent) + result
            for result in possible_results
        ]
        results_with_m = [
            result
            if throughput_check(result, available_throughput, timeslot_length)
            else MessageSet(messages.t_end, [], messages.t_end)
            for result in results_with_m
        ]
        possible_results = possible_results + results_with_m

    return max(
        possible_results,
        key=lambda msgs: context.utility(msgs)
    )


def adapt(messages, context, available_throughput, timeslot_length):
    return adapt_dynamic(
        messages, context, available_throughput, timeslot_length
    )


if __name__ == "__main__":
    messages, ctx = generate_simple_3D_reconstruction(8)

    simulate_sending_messages_with_latency(messages, 0.01)

    print(messages.__str__(
        ['sender', 'receivers', 't_gen', 't_sent', 't_rcv',
         'data_type_name']))

    print(ctx.utility_dict(messages))
    print(ctx.utility(messages))

    def available_throughput(t):
        return 2

    timeslot_length = 3

    from time import process_time

    start1 = process_time()
    actions1 = adapt_bruteforce(
        messages, ctx, available_throughput, timeslot_length
    )
    end1 = process_time()

    start2 = process_time()
    actions2 = adapt_dynamic(
        messages, ctx, available_throughput, timeslot_length
    )
    end2 = process_time()

    print("actions1 ({:.2f}s)".format(end1 - start1))
    print(actions1)
    print("actions2 ({:.2f}s)".format(end2 - start2))
    print(actions2)
    print(ctx.utility(actions1) == ctx.utility(actions2))
