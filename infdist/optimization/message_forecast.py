from heapq import heappush, heappop
import numpy as np

from .models import Message


class MessageForecast:
    def __init__(self, message_context):
        self.type_forecasts = {
            _type.data_type_name: _type.message_forecast_cls(
                **_type.message_forecast_kwargs
            )
            for _type in message_context.message_types
        }

    def register(self, message):
        self.type_forecasts[message.data_type_name].register_message(
            message
        )

    def estimate_t_end(self):
        return max([
            _type.estimate_t_end()
            for _type in self.type_forecasts.values()
        ])

    def message_generator(self, t_start, given_messages=None):
        if given_messages is None:
            given_messages = []

        heap = []
        for generator in [
            type_forecast.message_generator(
                t_start,
                [m for m in given_messages
                 if m.data_type_name == type_name]
            )
            for type_name, type_forecast in self.type_forecasts.items()
        ]:
            try:
                message = next(generator)
                heappush(
                    heap,
                    (message.t_gen, str(message.sender), message, generator)
                )
            except StopIteration:
                pass

        while heap:
            t_gen, sender, message, generator = heappop(heap)
            yield message
            try:
                new_message = next(generator)
                heappush(
                    heap,
                    (new_message.t_gen, str(new_message.sender),
                     new_message, generator)
                )
            except StopIteration:
                pass


class BaseTypeForecast:
    def register_message(self, message):
        raise NotImplementedError()

    def estimate_t_end(self):
        raise NotImplementedError()

    def message_generator(self, t_start, given_messages):
        raise NotImplementedError()


class FullKnowledgeTypeForecast(BaseTypeForecast):
    def __init__(self, messages, data_type_name):
        self.messages = messages
        self.data_type_name = data_type_name

    def register_message(self, message):
        pass

    def estimate_t_end(self):
        return self.messages.t_end

    def message_generator(self, t_start, given_messages):
        return iter(self.messages.filter(
            gen_after=t_start,
            data_type_name=self.data_type_name,
        ).all())


class EmptyTypeForecast(BaseTypeForecast):
    def __init__(self, t_end):
        self.t_end = t_end

    def register_message(self, message):
        pass

    def estimate_t_end(self):
        return self.t_end

    def message_generator(self, t_start, given_messages):
        for message in given_messages:
            yield message


class Samples:
    def __init__(self):
        self._aggregated = 0
        self.sample_size = 0

    def register(self, sample):
        self._aggregated += sample
        self.sample_size += 1

    def average(self):
        return self._aggregated/self.sample_size


class PeriodicTypeForecast(BaseTypeForecast):
    def __init__(
        self,
        data_type_name,
        t_end,
        max_depl_rate,
        battery_level,
        receivers,
        T,
        sender,
        size,
        initial_t_start=0,
    ):
        self.data_type_name = data_type_name
        self.t_end = t_end
        self.max_depl_rate = max_depl_rate
        self.battery_level = battery_level
        self.receivers = receivers
        self.T = T
        self.initial_t_start = initial_t_start
        self.t_start = initial_t_start
        self.t_start_samples = Samples()
        self.previous_message = None
        self.sender = sender
        self.size = size

    def register_message(self, message):
        assert self.sender == message.sender
        if self.previous_message is not None:
            t_start = message.t_gen % self.T
            self.t_start_samples.register(t_start)
            self.t_start = self.t_start_samples.average()

        self.previous_message = message

    def estimate_t_end(self):
        return self.t_end

    def create_message(self, t_gen):
        return Message(
            self.sender,
            set(self.receivers) - set([self.sender]),
            t_gen,
            self.data_type_name,
            self.size,
            {
                'battery_level': self.battery_level,
                'max_depl_rate': self.max_depl_rate,
            }

        )

    def message_generator(self, t_start, given_messages):
        if t_start < self.initial_t_start:
            t_start = self.initial_t_start
        start = t_start + self.T - (t_start - self.t_start) % self.T
        for t in np.arange(start, self.t_end, self.T):
            message = self.create_message(t)
            while given_messages and message.t_gen >= given_messages[0].t_gen:
                m = given_messages.pop(0)
                if m == message:
                    break
                yield m
            yield message

        for m in given_messages:
            yield m
