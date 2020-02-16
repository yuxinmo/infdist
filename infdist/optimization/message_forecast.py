from heapq import heappush, heappop


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
                heappush(heap, (message.t_gen, message, generator))
            except StopIteration:
                pass

        while heap:
            t_gen, message, generator = heappop(heap)
            yield message
            try:
                new_message = next(generator)
                heappush(heap, (new_message.t_gen, new_message, generator))
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
    def __init__(self, messages):
        self.messages = messages

    def register_message(self, message):
        print(f"skipping {message}")
        pass

    def estimate_t_end(self):
        return self.messages.t_end

    def message_generator(self, t_start, given_messages):
        return iter(self.messages.filter(gen_after=t_start).all())


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
